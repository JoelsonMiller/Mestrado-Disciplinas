%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ALGORITMO DE APRENDIZADO POR REFORÇO APLICADO A UM MANIPULADOR ROBÓTICO %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Comentários%%%%%%%%%%%%
% Scritp que apresentam bons resultados para o caso de regulação utilizando 
% ADHDP (com duas redes neurais (uma para o critico e outra para ator) com variações na carga (testado at� 5 kg)
% de trabalho e utilizando a seguinte função de ativação:
% tanh() e BP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
clc;
close all;

%% Iniciando as configurações para acesso remoto ao simulador v-rep.
sim=remApi('remoteApi'); %usando o arquivo remoteApi.m
sim.simxFinish(-1); % no caso de haver conexões abertas
clientID=sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5); 
%%%

%%
%%%%% Iniciando as variaveis necessárias %%%%%%
Ts = 0.001; %step da simulação

global w_ih_u;

%z_vector_size = 5; % tamanho do vetor zk

%nn_size_c = z_vector_size*(z_vector_size+1)/2;%78; %numero de neuronios da camada de escondida

global nn_size_a;
nn_size_a = 20;
global nn_size_c;
nn_size_c = 24;

state_size = 2;
input_size = 1;

Kp1 = 100;% Kp2 = 100; %ganho proporcional 
Kd1 = 20; %Kd2 = 20; %ganho derivativo 
ganhos = [Kp1, Kd1];%, Kp2, Kd2];

%Descomentar para carregar os pesos salvos
% w = load('weights_regulator/matlab.mat');
% ganhos = w.weights;

%q_func = qFunctionClass(z_vector_size, nn_size_c, ganhos); %Iniciando a Classe do função Q


%Matrizes de função de custo 
Qc = diag([10000*ones(1, state_size/2) 0.001*ones(1, state_size/2)]); %ponderação dos erros
R = 0.0001*diag(ones(1, input_size));   %ponderação do sinal de controle
S = 0.1*diag(ones(1, input_size));   %ponderação ???
%

n_joints = 6; % numero de juntas do manipulador
jointHandles = size(1,n_joints); % id de manipulação das juntas fornecida pelo simulador

q = zeros(1, n_joints); %iniciando a variavel de estados
dq = zeros(1, n_joints); %iniciando a variavel de estados
dsr_q = zeros(1,n_joints); %iniciando a variavel de estados desejados
dsr_dq =zeros(1,n_joints); %iniciando a variavel de estados desejados
tau_ant = zeros(1,n_joints); %iniciando a variavel do esforço de controle
eltr = zeros(1, nn_size_c); %iniciando o vetor de traços elegiveis
tau = tau_ant;
cost = 0;
edt = 0;
Qhat = 0;

power = 0.5; %Potência do ruído branco injetado no sinal de controle
Wc = zeros(nn_size_c,1); %iniciando o vetor de pesos do critico
Wc_ant = Wc;
Wa = zeros(nn_size_a, input_size);
wa_hat = Wa;

%hiperparametros
%alpha = 0.7; %fator de aprendizado
gamma = 0.95; %fator de desconto
beta = 10000; %valor para a matriz de covariancia da estimador RLS 
lambda = 0.85; %
mu = 1; %fator de esquecimento
epsilon = 1;
decay = 0.99999;
rate_learn = 1.5; %taxa de aprendizado do algoritmo NLMS

% Taxa de aprendizado dos gradientes d
alpha_c = 0.001;
alpha_a = 0.001;

% variaveis auxiliares
j = 1;
div = false;
result = false;
%
%Iniciando a matriz de covariancia
P = diag(beta*ones(1,nn_size_c)); % Iniciando a matrix de correlação

learning_cycle = 30; %segundos
n_cycles = 1;% numero de ciclos a serem testados na simulação

%%

if (clientID>-1)
    disp('Connected to remote API server');
    
    sim.simxSynchronous(clientID, true); %sincronização com o step da simulação
    
    %obtem os handles das juntas
    for i = 1:n_joints
        [rC,jointHandles(i)] = sim.simxGetObjectHandle(clientID,['UR10_joint' num2str(i)],sim.simx_opmode_blocking);
    end
    
    %configurando o step da simulação em sincronia com o controle
    sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step,Ts,sim.simx_opmode_oneshot);
    
    for i = 1:n_joints
       [r, q(i)] = sim.simxGetJointPosition(clientID, jointHandles(i), sim.simx_opmode_streaming);
       [r, dq(i)] = sim.simxGetObjectFloatParameter(clientID, jointHandles(i),2012, sim.simx_opmode_streaming);
    end
     
% Semente do ruído aleatorio
%semen = 2.121177000000000e+05; %Semente com bons resultados para o problema de regulação
semen = sum(100*clock); 
RandStream.setGlobalStream(RandStream('mt19937ar','seed',semen));  
format LONGE; 
%
    
%%   
%%%%%%%%%%%%%% INICIO DA SIMULAÇÃO %%%%%%%%%%%%%%%%%
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
    disp('Program is starting');
    k = 1; %passo de tempo
    
    while k <= n_cycles*learning_cycle/Ts %Inicio do processo iterativo
        
        if ~(mod(k,1000))
            disp(['Tempo: ', num2str(k)]);
        end
               
       %aplicar o sinal de controle no manipulador
        for i = 1:n_joints
            sim.simxSetJointTargetVelocity(clientID, jointHandles(i), sign(tau_ant(i))*10e10, sim.simx_opmode_oneshot);
            sim.simxSetJointForce(clientID, jointHandles(i), abs(tau_ant(i)), sim.simx_opmode_oneshot);
        end
        sim.simxSynchronousTrigger(clientID);
        
        %obtem a posição e a velocidade das juntas
        for i = 1:n_joints
           [r, q(i)] = sim.simxGetJointPosition(clientID, jointHandles(i), sim.simx_opmode_buffer);
           [r, dq(i)] = sim.simxGetObjectFloatParameter(clientID, jointHandles(i),2012, sim.simx_opmode_buffer);
        end
    
        % trajetoria a ser seguida
        path = desired_trajectory(k, Ts, 'regulator');
        dsr_q = [0 path(1) path(2) 0 0 0];
        dsr_dq = [0 path(3) path(4) 0 0 0];
        
        %calculo do erro de trajetória
        erro_q = q - dsr_q;
        erro_dq = dq - dsr_dq;
        %
       
        %Variaveis de Estado
        x = [q(3) dq(3)]; 
        e = [erro_q(3) erro_dq(3)];
        
        % Obtendo o ruído de exploração
       noise = noise_fnc(power(1), power(1), 'whitenoise', k); %ruído
        
       %Calculando a ação de controle 
       u = + Wa'*actv_func_a(x); %-Kp1*e(1) - Kd1*e(2);
       u = limit_u(u);
       tau(3) = u(1);     
        %
        if k > 1
            %Calculando o custo de controle
            cost = erro_ant*Qc*erro_ant' + u_ant*R*u_ant';% + ((tau-bufferU))*S*((tau-bufferU))';            
            
            % Target
            phi = actv_func_c(x,u);
            target = cost + gamma*Wc'*phi; 

            %Valor Ação-Estado
            phi_ant = actv_func_c(x_ant, u_ant);
            Qhat = Wc_ant'*phi_ant;    
            
            % Atualizando o vetor de traços elegiveis
            %eltr = lambda*gamma*eltr+bufferPhi;
            
            %Compute error
            %regression_vector = phi_ant - gamma*phi;
            %edt = cost - Wc'*regression_vector; %temporal diference
            edt = target - Qhat;
            
            Wc_ant = Wc;
            % Estimação dos pesos da rede neural do critico
            Wc = Wc - gamma*alpha_c*phi*(edt);
            
            wa_hat_ant = wa_hat;
            % Estimação dos pesos da rede neural  do ator            
            wa_hat = wa_hat - alpha_a*(Qhat)*actv_func_a(x)*(Wc_ant'*0.5*((1-phi.^2).* w_ih_u));
            
%            if k > 1000*j
                
                Wa = wa_hat;
%                 j = j+1;
%             end
            
            
            %%Verificando a convergência do parametros por meio da norma
            %%infinita

            if testConv(Wc, Wc_ant,1e-9) & k > 1000
                semen
                disp('CONVERGIU!!!');
                break;
            end
            
            % Limitando os pesos do rede neural do critico/Ator
%             if max(abs(Wc)) > 100
%                 Wc = Wc/max(abs(Wc)); 
%             end
%         
%             if max(abs(wa_hat)) > 100
%                 wa_hat = wa_hat/max(abs(wa_hat)); 
%             end
                                  
        end
  
        
         %% Salvando as variaveis para a proxima iteração
         erro_ant = e;
         tau_ant = tau;
         u_ant = u;
         x_ant = x;
         
         %salvando para plot
         stateplt(k,:) = x; %os estados
         controlplt(k) = u;% o esforço de controle
         pesos(k,:) = Wc'; %salvando os pesos para plot
         time(k) = k*Ts; 
         custos(k) = cost; %  Salvando os custos de para plot
         %policy_weights = get_policy_weights(q_func.weights);
         pesos_apply(k,:) = wa_hat;
         temporal_dif(k) = edt;
         if k <= 10
             q_value=0;
         else
            q_value(k) = target - Qhat - cost;
         end
         %
         q_hat(k) = Qhat;
         
        %
        k = k + 1;
        %
        
        % Verificar a divergência (sinal de controle muito alto)
        if isnan(u(1))
            disp('Divergência');
            disp(['Tempo ', num2str(k)])
            %div = true;
            tau =  zeros(1,n_joints);
            for i = 1:n_joints
                sim.simxSetJointTargetVelocity(clientID, jointHandles(i), sign(tau(i))*10e10, sim.simx_opmode_oneshot);
                sim.simxSetJointForce(clientID, jointHandles(i), abs(tau(i)), sim.simx_opmode_oneshot);
            end
            break;
        end 
                

    end
else
    disp('Failed connecting to remote API server');
end

sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot);
pause(1);

% PLT1 = {time, stateplt(:,1),'posição angular da junta 1', 'tempo (s)','angulo em rad'};
% PLT2 = {time, stateplt(:,2),'posição angular da junta 2', 'tempo (s)','angulo em rad'};
% PLT3 = {time, controlplt(:,1),'esforço de controle na junta 1', 'tempo (s)','torque em Nm'};
% PLT4 = {time, custos,'Custo de controle', 'tempo (s)',' '};
% plt = {PLT1, PLT2, PLT3, PLT4};
%plot_func(plt)

%% plotting
figure(1);
%subplot(211);
plot(time, rad2deg(stateplt(:,1)),'r','Linewidth',2),xlabel('time (s)'),ylabel('Trajetória Simulada (graus)')
title('Trajetória Simulada da Junta 1')
grid on
%legend('Trajetória Simulada da Junta 1')
% subplot(212);
% plot(time, rad2deg(stateplt(:,3)),'r','Linewidth',2),xlabel('time (s)'),ylabel('Velocidade Simulada (graus/s)')
% title('Velocidade Simulada da Junta 1')
% grid on
%legend('Velocidade Simulada da Junta 1')

 figure(2);
% %subplot(211);
 plot(time, rad2deg(stateplt(:,2)),'r','Linewidth',2),xlabel('time (s)'),ylabel('Trajetória Simulada (graus)')
 title('Trajetória Simulada da Junta 2')
 grid on
%legend('Trajetória Simulada da Junta 2')
% subplot(212);
% plot(time, rad2deg(stateplt(:,4)),'r','Linewidth',2),xlabel('time (s)'),ylabel('Velocidade Simulada (graus/2)')
% title('Trajetória Simulada da Junta 1')
% grid on
%legend('Trajetória Simulada da Junta 1')

figure(3);
plot(time,controlplt,'r',time, controlplt,'b','Linewidth',1),xlabel('time (s)'),ylabel('Sinal de controle aplicado')
title('Torque vs Time')
grid on

figure(4)
plot(time, pesos,'Linewidth',2),xlabel('time (s)'),ylabel('Pesos da Rede Neural')
title('Pesos da Rede Neural vs Time')

figure(5)
plot(time, custos,'k', time, q_value, 'r--', 'Linewidth',2),xlabel('time (s)'),ylabel(' ')
title('Custo de Rastreamento vs Q_value')

figure(6)
plot(time, pesos_apply,'Linewidth',2),xlabel('time (s)'),ylabel('Pesos da Rede Neural Aplicados')
title('Pesos da Rede Neural vs Time')

figure(7)
subplot(211);
plot(time, temporal_dif, 'LineWidth', 2), xlabel('time (s)'), ylabel(' ')
title('Erro de Diferença Temporal')
subplot(212);
plot(time, q_hat, 'LineWidth', 2), xlabel('time (s)'), ylabel(' ')
title('Q Value')
%%

sim.delete(); % call the destructor!

disp('Program ended');

function out = actv_func_c(x,u)

    global nn_size_c;
    global w_ih_u;
    rng(2)
    w_ih_x = -0.5 +1.*rand(nn_size_c, length(x));
    w_ih_u = -0.5 +1.*rand(nn_size_c, length(u));
    z = w_ih_x*x' + w_ih_u*u';
    out = ((1-exp(-z))./(1+exp(-z)));


end


function out = actv_func_a(x)

    global nn_size_a;
    rng(2)
    w_ih_x = -0.5 +1.*rand(nn_size_a, length(x));
    z = [w_ih_x*x'];
    out = ((1-exp(-z))./(1+exp(-z)));
    
end
    