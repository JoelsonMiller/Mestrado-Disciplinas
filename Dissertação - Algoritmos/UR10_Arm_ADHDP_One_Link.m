%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ALGORITMO DE APRENDIZADO POR REFORÇO APLICADO A UM MANIPULADOR ROBÓTICO %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Comentários%%%%%%%%%%%%

% Scritp que apresentam bons resultados para o caso de regulação utilizando 
% ADHDP (com uma rede neural para o critico e o ator) com variações na carga
% de trabalho e utilizando a seguinte função de ativação:
% 
% z = [u e e.^2]
% phi = kronecker(z);

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

z_vector_size = 5; % tamanho do vetor zk

nn_size = z_vector_size*(z_vector_size+1)/2;%78; %numero de neuronios da camada de escondida

state_size = 2;
input_size = 1;

Kp1 = 100;% Kp2 = 100; %ganho proporcional 
Kd1 = 20; %Kd2 = 20; %ganho derivativo 
ganhos = [Kp1, Kd1];%, Kp2, Kd2];

%Descomentar para carregar os pesos salvos
% w = load('weights_regulator/matlab.mat');
% ganhos = w.weights;

q_func = qFunctionClass(z_vector_size, nn_size, ganhos); %Iniciando a Classe do função Q


%Matrizes de função de custo 
Qc = diag([3000*ones(1, state_size/2) 0.001*ones(1, state_size/2)]); %ponderação dos erros
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
eltr = zeros(1, nn_size); %iniciando o vetor de traços elegiveis
tau = tau_ant;
cost = 0;
edt = 0;
Qhat = 0;

power = 1; %Potência do ruído branco injetado no sinal de controle
Wc = zeros(1, nn_size); %iniciando o vetor de pesos do critico


%hiperparametros
alpha = 0.6; %fator de aprendizado
gamma = 1; %fator de desconto
beta = 10000; %valor para a matriz de covariancia da estimador RLS 
lambda = 0.8; %
mu = 1; %fator de esquecimento
epsilon = 1;
decay = 0.99999;
rate_learn = 1.5; %taxa de aprendizado do algoritmo NLMS

% variaveis auxiliares
j = 1;
div = false;
result = false;
%
%Iniciando a matriz de covariancia
P = diag(beta*ones(1,nn_size)); % Iniciando a matrix de correlação

learning_cycle = 2; %segundos
n_cycles = 10;% numero de ciclos a serem testados na simulação

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
semen = 209949; %Semente com bons resultados para o problema de regulação
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
        
        
        %tau_ant = limit_u(tau_ant);
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
        dsr_q = [0 path(1) 0 path(2) 0 0];
        dsr_dq = [0 path(3) 0 path(4) 0 0];
        
        %calculo do erro de trajetória
        erro_q = q - dsr_q;
        erro_dq = dq - dsr_dq;
        %
       
        %Variaveis de Estado
        x = [q(3) dq(3)]; 
        e = [erro_q(3) erro_dq(3)];
        
        % Obtendo o ruído de exploração
       noise = noise_fnc(power(1), power(1), 'linear', k); %ruído
        
       %Calculando a ação de controle 
       u = q_func.politica(x,e) + noise(1);
       tau(3) = u(1);     
        %
        if k > 1
            %Calculando o custo de controle
            cost = erro_ant*Qc*erro_ant' + u_ant*R*u_ant';% + ((tau-bufferU))*S*((tau-bufferU))';
            
            
            % Target
            phi = q_func.kronecker(u, e, x);
            target = cost + gamma*q_func.weights*phi'; 

            %Valor Ação-Estado
            bufferPhi = q_func.kronecker(u_ant, erro_ant, x_ant);
            Qhat = Wc*bufferPhi';    
            
            % Atualizando o vetor de traços elegiveis
            eltr = lambda*gamma*eltr+bufferPhi;
            
            regression_vector = bufferPhi;% - lambda*phi;
            %Compute error
            %edt = cost - Wc*regression_vector'; %temporal diference
            edt = target - Qhat;
            
            %Regression vector
            rv = regression_vector';
            rls1 = P*eltr';
            rls2 = mu + rv'*rls1;
            K_RLS = rls1/rls2;
            %wk = wk + K_RLS'*edt;;
            
            Wc = estimator('RLS', K_RLS, Wc, edt,0);
            %wk = estimator('LMS', rate_learn, wk, edt, bufferPhi);
            %wk = estimator('gradient', 0.5, wk, edt,bufferPhi);
            %Atualizando a matriz de covariância
            P = (1/mu)*(P - (rls1*rv'*P)/rls2);
           
        end
        
        % Limitando os pesos do rede neural do critico/Ator
%         if abs(max(wk)) > 10000
%            wk = wk/abs(max(wk)); 
%         end
% 

        if min(eig(P))<1e-9
             P = diag(beta*ones(1,nn_size)); % Iniciando a matrix de correlação
        end
        
         if k == j*learning_cycle/Ts     
            P = diag(beta*ones(1,nn_size)); % Iniciando a matrix de correlação
            disp('Fim de um periodo de aprendizagem');
            %wk = limit_weights(wk);
            previous_weights = q_func.weights;
            q_func.weights = alpha*Wc + (1 - alpha)*q_func.weights;
            j = j + 1;
            
            %Verificando a convergência do parametros por meio da norma
            %infinita
            if testConv(q_func.weights, previous_weights,0.1)
                semen
                disp('CONVERGIU!!!');
                break;
            end
         end
        
         %% Salvando as variaveis para a proxima iteração
         erro_ant = e;
         tau_ant = tau;
         u_ant = u;
         x_ant = x;
         
         %salvando para plot
         stateplt(k,:) = x; %os estados
         controlplt(k) = u;% o esforço de controle
         pesos(k,:) = Wc; %salvando os pesos para plot
         time(k) = k*Ts; 
         custos(k) = cost; %  Salvando os custos de para plot
         policy_weights = get_policy_weights(q_func.weights);
         pesos_apply(k,:) = policy_weights;
         temporal_dif(k) = edt;
         if k <= 10
             q_value=0;
         else
            q_value(k) = Wc*regression_vector';
         end
         %
         q_hat(k) = Qhat;
         
        %
        k = k + 1;
        %
        
        % Verificar a divergência (sinal de controle muito alto)
        if abs(u(1)) > 10000
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
title('Erro de Diferença Temporal')
%%

sim.delete(); % call the destructor!

disp('Program ended');


    