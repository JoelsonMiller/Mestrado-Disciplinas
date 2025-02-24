%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ALGORITMO DE APRENDIZADO POR REFORÇO APLICADO A UM MANIPULADOR ROBÓTICO %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Comentários%%%%%%%%%%%%
% Scritp que apresentam bons resultados para o caso de regulação utilizando 
% ADHDP (com uma rede neural para o critico e outra para o ator) com variações na carga
% de trabalho ( testado com 2 kg) e utilizando RLS para o treinamento do
% critico, cuja função de ativação é:
%
% phi = kronecker(z); onde
%z = [u x x.^2];
%
%E utilizando o metodo do gradiente par o treinamento do ator utilizando
% a  função de ativação tanh
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

%z_vector_size = 9; % tamanho do vetor zk

%nn_size_c = z_vector_size*(z_vector_size+1)/2;%78; %numero de neuronios da camada de escondida

global nn_size_a;
nn_size_a = 36; %24
global nn_size_c;
nn_size_c = 55;%78; %numero de neuronios da camada de escondida;

state_size = 4;
input_size = 2;

Kp1 = 300; Kp2 = 200; %ganho proporcional 
Kd1 = 50; Kd2 =30; %ganho derivativo 
ganhos = [Kp1, Kd1, Kp2, Kd2];

%Descomentar para carregar os pesos salvos
% w = load('weights_regulator/matlab.mat');
% ganhos = w.weights;

%q_func = qFunctionClass(z_vector_size, nn_size_c, ganhos); %Iniciando a Classe do função Q


%Matrizes de função de custo 
Qc = diag([1000*ones(1, state_size/2) 0.001*ones(1, state_size/2)]); %ponderação dos erros
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
eltr = zeros(1, nn_size_c)'; %iniciando o vetor de traços elegiveis
tau = tau_ant;
cost = 0;
edt = 0;
Qhat = 0;

power = 20; %Potência do ruído branco injetado no sinal de controle
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
alpha_a = 0.0001;

% variaveis auxiliares
j = 1;
div = false;
result = false;
%
%Iniciando a matriz de covariancia
P = diag(beta*ones(1,nn_size_c)); % Iniciando a matrix de correlação

learning_cycle = 15; %segundos
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
semen = 2.124983000000000e+05; %Semente com bons resultados para o problema de regulação
%semen = sum(100*clock); 
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
        x = [q(2) q(3) dq(2) dq(3)]; 
        e = [erro_q(2) erro_q(3) erro_dq(2) erro_dq(3)];
        
        % Obtendo o ruído de exploração
       noise = noise_fnc(power(1), power(1), 'senoidal3', k); %ruído
        
       %Calculando a ação de controle 
       u = noise' - [Kp1*e(1)+Kd1*e(3) Kp2*e(2)+Kd2*e(4)]';%+Wa'*actv_func_a(x);
       u = limit_u(u);
       tau(2) = u(1);   
       tau(3) = u(2);
        %
        
        if k > 1
            %Calculando o custo de controle
            cost = erro_ant*Qc*erro_ant' + u_ant'*R*u_ant;% + ((tau-bufferU))*S*((tau-bufferU))';            
            
            % Target
            %phi = actv_func_c(x,u);
            phi = kronecker_c(x,u)';
            target = cost + gamma*Wc'*phi;

            %Valor Ação-Estado
            %phi_ant = actv_func_c(x_ant, u_ant);
            phi_ant = kronecker_c(x_ant,u_ant)';
            Qhat = Wc'*phi_ant;    
            
            % Atualizando o vetor de traços elegiveis
            %eltr = lambda*gamma*eltr+bufferPhi;
            
            %Compute error
            %regression_vector = phi_ant;% - gamma*phi;
            %edt = cost - Wc'*regression_vector; %temporal diference
            edt = target - Qhat;
            
            %Regression vector
            rv = phi_ant- gamma*phi;
            rls1 = P*rv;
            rls2 = mu + rv'*rls1;
            K_RLS = rls1/rls2;
            
            Wc_ant = Wc;
            Wc = estimator('RLS', K_RLS', Wc, edt,0);
            
            %Atualizando a matriz de covariância
            P = (1/mu)*(P - (rls1*rv'*P)/rls2);
            
            if mod(k,2000) == 0               
                P = diag(beta*ones(1,nn_size_c)); % Re-Iniciando a matrix de correlação
            end
            
            
            % Estimação dos pesos da rede neural do critico
            %Wc = Wc - gamma*alpha_c*phi*(edt);
                                              
            % Estimação dos pesos da rede neural  do ator    
            %wa_hat_ant = wa_hat;
            %wa_hat = wa_hat - alpha_a*(Wc_ant'*phi)*actv_func_a(x)*(Wc_ant'*0.5*((1-phi.^2).* w_ih_u));
             %phi_dot = [2*u x x.^2 cos(x) cos(x).^2 zeros(1,nn_size_c - z_vector_size)]';
             phi1 = [2*u(1) u(2) x x.^2 zeros(1, 45)];
             phi2 = [0 u(1) zeros(1,8) 2*u(2) x x.^2 zeros(1,36)];
             phi_dot = [phi1' phi2'];
             wa_hat = wa_hat - alpha_a*(Wc_ant'*phi)*actv_func_a(x)*(Wc_ant'*phi_dot);
            
           % if k > 1000*j
                
                Wa = wa_hat;
             %    j = j+1;
             %end
            
            
            %%Verificando a convergência do parametros por meio da norma
            %%infinita

            if testConv(Wc, Wc_ant,1e-9) & k > 1000
                semen
                disp('CONVERGIU!!!');
                break;
            end
            
            % Limitando os pesos do rede neural do critico/Ator
%             if max(abs(Wc)) > 1000
%                 Wc = Wc/max(abs(Wc)); 
%             end
        
            if max(abs(wa_hat(:,1))) > 1000
                wa_hat(:,1) = wa_hat(:,1)/max(abs(wa_hat(:,1))); 
            end
            
            if max(abs(wa_hat(:,2))) > 1000
                wa_hat(:,2) = wa_hat(:,2)/max(abs(wa_hat(:,2))); 
            end
                                  
        end
  
        
         %% Salvando as variaveis para a proxima iteração
         erro_ant = e;
         tau_ant = tau;
         u_ant = u;
         x_ant = x;
         
         %salvando para plot
         stateplt(k,:) = x; %os estados
         controlplt(k,:) = u';% o esforço de controle
         pesos(k,:) = Wc'; %salvando os pesos para plot
         time(k) = k*Ts; 
         custos(k) = cost; %  Salvando os custos de para plot
         %policy_weights = get_policy_weights(q_func.weights);
         pesos_apply(k,:) = [wa_hat(:,1); wa_hat(:,2)]';
         temporal_dif(k) = edt;
         if k <= 10
             q_value=0;
         else
            q_value(k) = Wc'*(phi_ant - gamma*phi);
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
subplot(211);
plot(time, rad2deg(stateplt(:,1)),'r','Linewidth',2),xlabel('time (s)'),ylabel('Trajetória Simulada (graus)')
title('Trajetória Simulada da Junta 1')
grid on
legend('Trajetória Simulada da Junta 1')
subplot(212);
plot(time, rad2deg(stateplt(:,3)),'r','Linewidth',2),xlabel('time (s)'),ylabel('Velocidade Simulada (graus/s)')
title('Velocidade Simulada da Junta 1')
grid on
legend('Velocidade Simulada da Junta 1')

 figure(2);
subplot(211);
 plot(time, rad2deg(stateplt(:,2)),'r','Linewidth',2),xlabel('time (s)'),ylabel('Trajetória Simulada (graus)')
 title('Trajetória Simulada da Junta 2')
 grid on
legend('Trajetória Simulada da Junta 2')
subplot(212);
plot(time, rad2deg(stateplt(:,4)),'r','Linewidth',2),xlabel('time (s)'),ylabel('Velocidade Simulada (graus/2)')
title('Trajetória Simulada da Junta 1')
grid on
legend('Trajetória Simulada da Junta 1')

figure(3);
plot(time,controlplt(:,1),'r',time, controlplt(:,2),'b','Linewidth',1),xlabel('time (s)'),ylabel('Sinal de controle aplicado')
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
    rng(100)
    w_ih_x = -0.3 + 0.6*rand(nn_size_c, length(x));
    w_ih_u = -0.3 + 0.6*rand(nn_size_c, length(u));
    z = w_ih_x*x' + w_ih_u*u;
    out = ((1-exp(-z))./(1+exp(-z)));

end


function out = actv_func_a(x)

    global nn_size_a;
    rng(2)
    w_ih_x = -0.3 + 0.6*rand(nn_size_a, length(x));
    z = [w_ih_x*x'];
    out = ((1-exp(-z))./(1+exp(-z)));
    
end
  

function phi = kronecker_c(inp_x, inp_u)
    x = [inp_u' inp_x inp_x.^2];

    
    index = 1;
    for i=1:length(x)
        aux = x(i);
        for j=1:length(x)
            if j>=i
                phi(index) = aux*x(j);
                index = index + 1;
            end
        end
    end
end


function phi_a = kronecker_a(inp_x)

    x = [inp_x inp_x.^2 cos(inp_x) cos(inp_x).^2];
    
    index = 1;
    for i=1:length(x)
        aux = x(i);
        for j=1:length(x)
            if j>=i
                phi_a(index) = aux*x(j);
                index = index + 1;
            end
        end
    end


end