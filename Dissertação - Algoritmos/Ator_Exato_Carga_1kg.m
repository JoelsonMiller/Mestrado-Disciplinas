%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ALGORITMO DE APRENDIZADO POR REFORÇO APLICADO A UM MANIPULADOR ROBÓTICO %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
clc;
close all;

 index = 6000;
% 
 %testes = 15;
% 
 gammas = [0.1 0.15 0.2 0.25];
% 
% lambdas = [0.9 0.92 0.94 0.96 0.98 1];
% 
% learn_cycles = [1 1.5 2 2.5 3 4];

%powers = [1 2 3 4 5 10];

%  for l = 1:length(lambdas)
 % for g = 1:length(gammas)
%  for lc = 1:length(learn_cycles)
% % for p = 1:length(powers)
  %for t = 1:testes

%disp(['Teste de nu: ' num2str(t)]);
    
%% Iniciando as configurações para acesso remoto ao simulador v-rep.
sim=remApi('remoteApi'); %usando o arquivo remoteApi.m
sim.simxFinish(-1); % no caso de haver conexões abertas
clientID=sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5); 
%%%

%%
%%%%% Iniciando as variaveis necessárias %%%%%%
Ts = 0.005; %step da simulação
z_vector_size = 14; % tamanho do vetor zk

nn_size = z_vector_size*(z_vector_size+1)/2;%78; %numero de neuronios da camada de escondida

state_size = 4;
input_size = 2;

Kp1 = 500; Kp2 = 500; %ganho proporcional 500 e 300
Kd1 = 50; Kd2 = 50; %ganho derivativo  50 e 30
ganhos = [Kp1, Kd1, Kp2, Kd2];

%global md;

%md = load('multi_demanda.mat');

%Descomentar para carregar os pesos salvos
%  w = load('pesos_treinados/unit_step_exact_actor_carga.mat');
%  ganhos = w.Wc;

q_func = qFunctionClass(z_vector_size, nn_size, ganhos); %Iniciando a Classe do função Q

%Matrizes de função de custo 
Qc = diag([200*ones(1, state_size/2) 0.001*ones(1, state_size/2)]); %ponderação dos erros
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
tau = tau_ant;
ic = zeros(1, 2); 
cost = 0;
edt = 0;
Qhat = 0;

power = [0.5, 0.5];%[powers(p), powers(p)]; %Potência do ruído branco injetado no sinal de controle
Wc = zeros(1, nn_size); %iniciando o vetor de pesos do critico

%hiperparametros
gamma = 0.5;%gammas(g); %0.4 %fator de aprendizado
lambda = 0.94;%lambdas(l);%0.98; %fator de desconto
beta = 10000; %valor para a matriz de covariancia da estimador RLS 
mu = 1; %fator de esquecimento
epsilon = 1;
decay = 0.99999;
rate_learn = 1.5; %taxa de aprendizado do algoritmo NLMS

% variaveis auxiliares
j = 1;
p = 1;
div = false;
result = false;
%
%Iniciando a matriz de covariancia
P = diag(beta*ones(1,nn_size)); % Iniciando a matrix de correlação
%
learning_cycle = 1.5;%learn_cycles(lc); %segundos
n_cycles = 13;% numero de ciclos a serem testados na simulação

%%

if (clientID>-1)
    disp('Connected to remote API server');
    
    sim.simxSynchronous(clientID, true); %sincronização com o step da simulação
    
    %obtem os handles das juntas
    for i = 1:n_joints
        [rC,jointHandles(i)] = sim.simxGetObjectHandle(clientID,['UR10_joint' num2str(i)],sim.simx_opmode_blocking);
    end    

     %obter o handle da carga de trabalho
    [rC,payloadHandle] = sim.simxGetObjectHandle(clientID,'payload',sim.simx_opmode_blocking);
    sim.simxSetObjectFloatParameter(clientID, payloadHandle, 3005, 0.0001, sim.simx_opmode_streaming);
    %
    
    %configurando o step da simulação em sincronia com o controle
    sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step,Ts,sim.simx_opmode_oneshot);
    
    for i = 1:n_joints
       [r, q(i)] = sim.simxGetJointPosition(clientID, jointHandles(i), sim.simx_opmode_streaming);
       [r, dq(i)] = sim.simxGetObjectFloatParameter(clientID, jointHandles(i),2012, sim.simx_opmode_streaming);
    end
     
% Semente do ruído aleatorio
semen = 2.132676000000000e+05; %2.119572000000000e+05; %2.105456000000000e+05;%2.119572000000000e+05; %2.084214000000000e+05;%2.095804000000000e+05;%2.087297000000000e+05;%2.176106000000000e+05;%%2.133121000000000e+05;%2.111926000000000e+05; %Semente com bons resultados para o problema de regulação
%semen = sum(100*clock) 
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
            sim.simxSetJointTargetVelocity(clientID, jointHandles(i), sign(tau_ant(i))*rad2deg(360), sim.simx_opmode_oneshot);
            sim.simxSetJointForce(clientID, jointHandles(i), abs(tau_ant(i)), sim.simx_opmode_oneshot);
        end
        sim.simxSynchronousTrigger(clientID);
        
        %obtem a posição e a velocidade das juntas
        for i = 1:n_joints
           [r, q(i)] = sim.simxGetJointPosition(clientID, jointHandles(i), sim.simx_opmode_buffer);
           [r, dq(i)] = sim.simxGetObjectFloatParameter(clientID, jointHandles(i),2012, sim.simx_opmode_buffer);
        end
    
%         if q(2) == 43
%            q = [0 0 0 0 0 0]; 
%         end
        
        % trajetoria a ser seguida
        
        path = desired_trajectory(k, Ts, 'unitstep');
        dsr_q = [0 path(1) path(2) 0 0 0];
        dsr_dq = [0 path(3) path(4) 0 0 0];
        
        %calculo do erro de trajetória
        erro_q = q - dsr_q;
        erro_dq = dq - dsr_dq;
        %
       
        %Variaveis de Estado
        x = [q(2) q(3) dq(2) dq(3)]; 
        e = [erro_q(2) erro_q(3) erro_dq(2) erro_dq(3)];
        
        %noise =  wgn(1,1,1,'linear');
       %noise = power*randn();
       % if n_cycles*learning_cycle/(2*Ts) >= k
             noise = noise_fnc(power(1), power(2), 'whitenoise', k); %ruído
       % else
         %  noise = noise_fnc(1, 1, 'linear', k);
       %end
        %taxa de decaimento do ruído de exploração.
        %um valor minimo para o ruído é estabelecido para satisfazer o
        %condição de excitação persistente
%       if abs(power) >= 5 & (mod(2,k) == 0)
%           power = power*decay;
%       end
        
       %Calculando a ação de controle 

       u = q_func.politica(x,e) + noise;%- [150*e(1)+30*e(3) 150*e(2)+30*e(4)]+noise;%  

       u = limit_u(u);
       tau(2) = u(1); 
       tau(3) = u(2);
        %
        if k > 5
            %Calculando o custo de controle
            cost = erro_ant*Qc*erro_ant' + u_ant*R*u_ant';% + ((tau-bufferU))*S*((tau-bufferU))'; 
            % Target
            phi = q_func.kronecker(u, e, x);
            target = cost + lambda*q_func.weights*phi'; 

            %Valor Ação-Estado

            bufferPhi = q_func.kronecker(u_ant, erro_ant, x_ant);
            Qhat = Wc*bufferPhi';    
            
            regression_vector = bufferPhi;% - lambda*phi;
            %Compute error
            %edt = cost - Wc*regression_vector'; %temporal diference
            edt = target - Qhat;
            
            %edt = min(1000, max(edt,-1000));
            
            %Regression vector
            rv = regression_vector';
            rls1 = P*rv;
            rls2 = mu + rv'*rls1;
            K_RLS = rls1/rls2;
            %q_valuewk = wk + K_RLS'*edt;;
            
            %edt = min(1000, max(edt,-1000));
            Wc = estimator('RLS', K_RLS, Wc, edt,0);

                %wk = estimator('LMS', rate_learn, wk, edt, bufferPhi);
            %wk = estimator('gradient', 0.5, wk, edt,bufferPhi);
            %Atualizando a matriz de covariância
            P = (1/mu)*(P - (rls1*rv'*P)/rls2);
           
%             if min(eig(P))<1e-9
%                 P = diag(beta*ones(1,nn_size)); % Iniciando a matrix de correlação
%             end
        end
        
%          if max(abs(Wc)) > 5000
%             Wc = Wc/max(abs(Wc)); 
%          end
%               
         if k == cast(j*learning_cycle/Ts, 'int32')     
            P = diag(beta*ones(1,nn_size)); % Re-Iniciando a matrix de correlação
            disp('Fim de um periodo de aprendizagem');
         %end
            %wk = limit_weights(wk);
            previous_weights = q_func.weights;
            q_func.weights = gamma*Wc + (1 - gamma)*q_func.weights;
            %result  = test_convergence(q_func.weights, previous_policy, epsilon);
            j = j + 1;
            ic = [tau_ant(2) tau_ant(3)];
            %Verificando a convergência do parametros por meio da norma
            %infinita
            if testConv(q_func.weights, previous_weights,0.01)
                semen
                disp('CONVERGIU!!!');
                %break;
            end
         end
        
         if k == 10/Ts %& k<=4000
            %mass = 4*(k-2000)/2000+1
            mass = 1;
            disp('Adicionando uma carga de 5 kg');
            sim.simxSetObjectFloatParameter(clientID, payloadHandle, 3005, mass, sim.simx_opmode_oneshot);
         end
         
         
         [rC,massValue(k)]=sim.simxGetObjectFloatParameter(clientID, payloadHandle, 3005, sim.simx_opmode_oneshot);
         
         q_function(k) = Qhat;
         
         if k > 3200
            gamma = 0.05; 
         end
         
         %% Salvando as variaveis para a proxima iteração
         erro_ant = e;
         tau_ant = tau;
         u_ant = u;
         x_ant = x;
         
         %salvando para plot
         stateplt(k,:) = x; %os estados
         dsrplt(k,:) = [path(1) path(2) path(3) path(4)];
         controlplt(k,:) = u;% o esforço de controle
         pesos(k,:) = Wc; %salvando os pesos para plot
         time(k) = k*Ts; 
         custos(k) = cost; %  Salvando os custos de para plot
         policy_weights = get_policy_weights(q_func.weights);
         pesos_apply(k,:) = policy_weights;
         temporal_dif(k) = edt;
         if k <=10
             q_value=0;
         else
            q_value(k) = Qhat - target + cost;%Wc*regression_vector'; %
         end
         %
         
        %
        k = k + 1;
        %
        
        % Verificar a divergência (sinal de controle muito alto)
%         if abs(u(1)) > 10000 || isnan(u(1))
%             disp('Divergência');
%             disp(['Tempo ', num2str(k)])
%             %div = true;
%             tau =  zeros(1,n_joints);
%             for i = 1:n_joints
%                 sim.simxSetJointTargetVelocity(clientID, jointHandles(i), sign(tau(i))*10e10, sim.simx_opmode_oneshot);
%                 sim.simxSetJointForce(clientID, jointHandles(i), abs(tau(i)), sim.simx_opmode_oneshot);
%             end
%             break;
%         end 
                

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

%pd = load('plots/state_pd_step_payload.mat');
%s_pd = rad2deg(pd.stateplt);

f(1) = figure(1);
subplot(211);
%plot(time, s_pd(:,1), 'b','Linewidth',2);
hold on
plot(time, rad2deg(dsrplt(:,1)), 'k',  time, rad2deg(stateplt(:,1)),'r','Linewidth',2); hold off
xlabel('time (s)'),ylabel('Trajetória Simulada (graus)')
title('Trajetória Simulada da Junta 1')
grid on
legend('Trajetória Simulada da Junta 1')
ylim([-2 10])
xlim([9.5 n_cycles*learning_cycle])
subplot(212);
%plot(time, s_pd(:,3), 'b','Linewidth',2);
hold on
plot(time,  rad2deg(dsrplt(:,3)), 'k', time, rad2deg(stateplt(:,3)),'r','Linewidth',2); hold off
xlabel('time (s)'),ylabel('Velocidade Simulada (graus/s)')
title('Velocidade Simulada da Junta 1')
grid on
legend('Velocidade Simulada da Junta 1')


f(2) = figure(2);
subplot(211);
%plot(time, s_pd(:,2), 'b','Linewidth',2);
hold on
plot(time,  rad2deg(dsrplt(:,2)), 'k', time, rad2deg(stateplt(:,2)),'r','Linewidth',2),xlabel('time (s)'),ylabel('Trajetória Simulada (graus)')
hold off
title('Trajetória Simulada da Junta 2')
grid on
legend('Trajetória Simulada da Junta 2')
ylim([88 100])
xlim([9.5 n_cycles*learning_cycle])
subplot(212);
%plot(time, s_pd(:,4), 'b','Linewidth',2);
hold on
plot(time,  rad2deg(dsrplt(:,4)), 'k',time,  rad2deg(stateplt(:,4)),'r','Linewidth',2),xlabel('time (s)'),ylabel('Velocidade Simulada (graus/2)')
hold off
title('Trajetória Simulada da Junta 1')
grid on
legend('Trajetória Simulada da Junta 1')

f(3) = figure(3);
plot(time,controlplt(:,1),'r',time, controlplt(:,2),'b','Linewidth',1),xlabel('time (s)'),ylabel('Sinal de controle aplicado')
title('Torque vs Time')
grid on

f(4) = figure(4);
plot(time, pesos,'Linewidth',2),xlabel('time (s)'),ylabel('Pesos da Rede Neural')
title('Pesos da Rede Neural vs Time')

f(5) = figure(5);
plot(time, custos,'k', time, q_value, 'r--', 'Linewidth',2),xlabel('time (s)'),ylabel(' ')
title('Custo de Rastreamento vs Q_value')

f(6) = figure(6);
plot(time, pesos_apply,'Linewidth',2),xlabel('time (s)'),ylabel('Pesos da Rede Neural Aplicados')
title('Pesos da Rede Neural vs Time')

f(7) = figure(7);
plot(time, temporal_dif, 'LineWidth', 2), xlabel('time (s)'), ylabel(' ')
title('Erro de Diferença Temporal')

f(8) = figure(8)
plot(time, massValue, 'LineWidth', 2), xlabel('time (s)'), ylabel(' ')
title('Massa da Carga de Trabalho')

f(9)  = figure(9)
plot(time, q_function, 'LineWidth', 2), xlabel('time (s)'), ylabel(' ')
title('Valores Q')

% disp(index);
%savefig(f, ['TESTES/test' num2str(index) '.fig']);
% close(f);
% index = index + 1;
% 
% clear stateplt
% clear controlplt
% clear pesos
% clear time 
% clear custos
% clear pesos_apply
% clear temporal_dif
% clear q_value
% clear q_hat
%  end
% end
% end
% end
%end
%%

sim.delete(); % call the destructor!

disp('Program ended');


    