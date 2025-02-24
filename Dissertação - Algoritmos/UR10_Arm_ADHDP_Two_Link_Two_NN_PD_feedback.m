%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ALGORITMO DE APRENDIZADO POR REFORÇO APLICADO A UM MANIPULADOR ROBÓTICO %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Comentários%%%%%%%%%%%%
% Script para testes do algoritmo de aprendizado por reforço (ADHDP) com
% duas NN (para o critico e o ator) para a tarefa de rastreamento com uma
% malha PD sem modificação na carga de trabalho
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
clc;
close all;

index = 1;

%action_nn = [4 6 8 12 16 20 24 28];

n_testes = 10;

%weights_limits = [100 200 500 1000 2000];

weight_error = [100 150 200 250 300 400 500];
    
% for act = 1:length(action_nn)
%     
% %for wgts_lmt = 1:length(weights_limits)
%     
%  for wgts_e = 1:length(weight_error)
%     
%  for test = 1:n_testes
% 
%  disp(['Teste de num ' num2str(test)]);    

%% Iniciando as configurações para acesso remoto ao simulador v-rep.
sim=remApi('remoteApi'); %usando o arquivo remoteApi.m
sim.simxFinish(-1); % no caso de haver conexões abertas
clientID=sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5); 
%%%

%%
%%%%% Iniciando as variaveis necessárias %%%%%%
Ts = 0.005; %step da simulação

global w_ih_u;

%z_vector_size = 9; % tamanho do vetor zk

%nn_size_c = z_vector_size*(z_vector_size+1)/2;%78; %numero de neuronios da camada de escondida

global nn_size_a;
nn_size_a = 12;%action_nn(act); %48
global nn_size_c;
nn_size_c = 55; %numero de neuronios da camada de escondida;

state_size = 4;
input_size = 2;

Kp1 = 500; Kp2 = 500; %ganho proporcional 
Kd1 = 50; Kd2 = 50; %ganho derivativo 
ganhos = [Kp1, Kd1, Kp2, Kd2];

%Descomentar para carregar os pesos salvos
w = load('action_weights.mat');
c = load('critic_weights.mat');
%ganhos = w.weights;

%q_func = qFunctionClass(z_vector_size, nn_size_c, ganhos); %Iniciando a Classe do função Q


%Matrizes de função de custo 
q12 = 50;%weight_error(wgts_e);
Qc = diag([q12*ones(1, state_size/2) 0.001*ones(1, state_size/2)]); %ponderação dos erros
R = 0.00001*diag(ones(1, input_size));   %ponderação do sinal de controle
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
e_ant = [0 0 0 0];
e_ant_2 = [0 0 0 0];
u_pd = [0;0];

power = 5; %Potência do ruído branco injetado no sinal de controle
Wc = c.Wc;%-1 + 2*rand(nn_size_c,1); %iniciando o vetor de pesos do critico
Wc_ant = Wc;
Wa = w.Wa;%-1 + 2*rand(nn_size_a, input_size);
wa_hat = Wa;

%hiperparametros
%alpha = 0.7; %fator de aprendizado
gamma = 0.9; %fator de desconto
beta = 10000; %valor para a matriz de covariancia da estimador RLS 
lambda = 0.85; %
mu = 1; %fator de esquecimento
epsilon = 1;
decay = 0.99999;
rate_learn = 1.5; %taxa de aprendizado do algoritmo NLMS

% Taxa de aprendizado dos gradientes d
alpha_c = 0.001;
alpha_a = [0.01 0.01];

% variaveis auxiliares
j = 1;
div = false;
result = false;
%
%Iniciando a matriz de covariancia
P = diag(beta*ones(1,nn_size_c)); % Iniciando a matrix de correlação

learning_cycle = 80; %segundos
n_cycles = 1;% numero de ciclos a serem testados na simulação


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
    %sim.simxSetObjectFloatParameter(clientID, payloadHandle, 3005, 0.0001, sim.simx_opmode_streaming);
    
    %configurando o step da simulação em sincronia com o controle
    sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step,Ts,sim.simx_opmode_oneshot);
    
    for i = 1:n_joints
       [r, q(i)] = sim.simxGetJointPosition(clientID, jointHandles(i), sim.simx_opmode_streaming);
       [r, dq(i)] = sim.simxGetObjectFloatParameter(clientID, jointHandles(i),2012, sim.simx_opmode_streaming);
    end
   
% Semente do ruído aleatorio
semen = 2.094451000000000e+05;%2.148281000000000e+05;%2.159309000000000e+05;%2.102154000000000e+05;%2.124983000000000e+05; %Semente com bons resultados para o problema de regulação
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
%         
%         if ~(mod(k,1000))
%             disp(['Tempo: ', num2str(k)]);
%         end
               
       %aplicar o sinal de controle no manipulador
        for i = 1:n_joints
            sim.simxSetJointTargetVelocity(clientID, jointHandles(i), sign(tau_ant(i))*deg2rad(360), sim.simx_opmode_oneshot);
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
        er = [x dsr_q(2) dsr_q(3) dsr_dq(2) dsr_dq(3)];
        
        % Obtendo o ruído de exploração
       noise = noise_fnc(power(1), power(1), 'whitenoise', k); %ruído
        
       %Calculando a ação de controle 
       u_rl = Wa'*actv_func_a(e) + noise'; %- [Kp1*e(1)+Kd1*e(3) Kp2*e(2)+Kd2*e(4)]';
       %u_pd = [Kp1*(e(1)-e_ant(1))+Kd1*(e(3)-2*e_ant(3)+e_ant_2(3)) Kp2*(e(2)-e_ant(2))+Kd2*(e(4)-2*e_ant(4)+e_ant_2(4))]'+u_pd;
       u_pd = - [Kp1*e(1)+Kd1*e(3) Kp2*e(2)+Kd2*e(4)]';
       u = u_rl; %limit_u(u_rl+u_pd);
       tau(2) = min(330, max(u(1),-330));   
       tau(3) = min(330, max(u(2),-330));
        %
        
        if k > 1
            %Calculando o custo de controle
            cost = (erro_ant*Qc*erro_ant' + u_ant'*R*u_ant);% + ((tau-bufferU))*S*((tau-bufferU))';            
            
            % Target
            %phi = actv_func_c(e,u);
            
            phi = kronecker_c(e,u)';
            target = cost + gamma*Wc'*phi;

            %Valor Ação-Estado
            %phi_ant = actv_func_c(e_ant, u_ant);
            phi_ant = kronecker_c(e_ant,u_ant)';
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
            
%             if mod(k, 2000) == 0
%                 %q12 =  q12 + 10;
%                 P = diag(beta*ones(1,nn_size_c)); % Re-Iniciando a matrix de correlação
%                 %Qc = diag([q12*ones(1, state_size/2) 0*ones(1, state_size/2)]); 
%             end
                       
            % Estimação dos pesos da rede neural do critico
%             phi = actv_func_c(e,u);
%             phi_ant = actv_func_c(e_ant,u_ant);
%             Qhat = Wc'*phi_ant;
%             edt = cost+lambda*Wc'*phi - Qhat;
%             Wc_ant = Wc;
%             Wc = Wc - gamma*alpha_c*phi*(edt);
                                              
            % Estimação dos pesos da rede neural  do ator    
%             wa_hat_ant = wa_hat;
%             wa_hat = wa_hat - alpha_a.*((Wc_ant'*phi)*actv_func_a(x)*(Wc_ant'*0.5*((1-phi.^2).* w_ih_u)));
             
             phi1 = [2*u(1) u(2) e e.^2 zeros(1, 45)];
             phi2 = [0 u(1) zeros(1,8) 2*u(2) e e.^2 zeros(1,36)];
             phi_dot = [phi1' phi2'];
             phi_a = actv_func_a(e);
             erro_a = min(100, max(Wc_ant'*phi,-100));
             delta_wa = - alpha_a.*((Wc_ant'*phi)*phi_a*(Wc_ant'*phi_dot));
             wa_hat = Wa + delta_wa;
            
            %Treinando do Controlador PD
             
             %Wa = Wa - 0.0001*actv_func_a(e)*(u_rl-u_pd)';
                         
            %%Verificando a convergência do parametros por meio da norma
            %%infinita

%             if testConv(Wc, Wc_ant,1e-6) & k > 1000
%                 semen
%                 disp('CONVERGIU!!!');
%                 %break;
%             end
            
            %Limitando os pesos do rede neural do critico/Ator
%             if max(abs(Wc)) > 1000
%                 Wc = Wc/max(abs(Wc)); 
%             end              
% %             
            if max(abs(wa_hat(:,1))) > 500 %weights_limits(wgts_lmt)
                wa_hat(:,1) = wa_hat(:,1)/max(abs(wa_hat(:,1))); 
            end
            
            if max(abs(wa_hat(:,2))) > 500 %weights_limits(wgts_lmt)
                wa_hat(:,2) = wa_hat(:,2)/max(abs(wa_hat(:,2))); 
            end

%              if k > 2000    
%                  disp('adicionando o sinal de controle do ator');
                   %Wa = wa_hat;
%                  %j = j+1;
%               end
%                                    
        end
  
%          if k == 8000
%             disp('Adicionando uma carga de 5 kg');
%             sim.simxSetObjectFloatParameter(clientID, payloadHandle, 3005, 5.0, sim.simx_opmode_oneshot);
%         end
%             

    % Atualizando a taxa de aprendizado da rede do ator
%      decay = (1 - k/(learning_cycle/n_cycles)); 
%      alpha_a = alpha_a*decay;
    %
         %% Salvando as variaveis para a proxima iteração
         erro_ant = e;
         tau_ant = tau;
         u_ant = [tau(2) tau(3)]';
         e_ant_2 = e_ant;
         e_ant = e;
         er_ant = er;
         
         
         %salvando para plot
         stateplt(k,:) = e; %os erros
         controlplt(k,:) = [tau(2) tau(3)];% o esforço de controle
         control_rl(k,:) = u_rl'; %saída do ator
         pesos(k,:) = Wc'; %salvando os pesos para plot
         time(k) = k*Ts; 
         custos(k) = cost; %  Salvando os custos de para plot
         %policy_weights = get_policy_weights(q_func.weights);
         pesos_apply(k,:) = [Wa(:,1); Wa(:,2)]';
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
f(1) = figure(1);
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

f(2) = figure(2);
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

f(3) = figure(3);
subplot(211);
plot(time,controlplt(:,1),'r',time, controlplt(:,2),'b','Linewidth',1),xlabel('time (s)'),ylabel('Sinal de controle aplicado')
title('Torque vs Time')
subplot(212);
plot(time,controlplt(:,1),'r',time, control_rl(:,2),'b','Linewidth',1),xlabel('time (s)'),ylabel('Sinal de controle aplicado')
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
subplot(211);
plot(time, temporal_dif, 'LineWidth', 2), xlabel('time (s)'), ylabel(' ')
title('Erro de Diferença Temporal')
subplot(212);
plot(time, q_hat, 'LineWidth', 2), xlabel('time (s)'), ylabel(' ')
title('Q Value')
grid on

% f(8) = figure(8);
% plot(time, var, 'LineWidth', 2), xlabel('time (s)'), ylabel(' ')
% title('Sinal de saída da camada escondida do ator')
% grid on


% save('action_weights.mat', 'Wa');
% save('critic_weights.mat', 'Wc');

% 
% disp(index);
% savefig(f, ['TESTES/testes_25_02' num2str(index) '.fig']);
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
% 
% end
% end
% end
%end
%%

sim.delete(); % call the destructor!

disp('Program ended');

function out = actv_func_c(x,u)

%     b1 = 2.909987461850145;
%     a1 = -b1;
%     
%     b2 = 45;
%     a2 = -b2;
% 
%     xn1 = -1 + ([x(1) x(2)]-a1)*2/(b1-a1);
%     xn2 = -1 + ([x(3) x(4)]-a2)*2/(b2-a2);
%     
%     xn = [xn1 xn2];
% 
%     b3 = 150;
%     a3 = -150;
%     
%     un = -1 +  (u - a3)*2/(b3-a3);
    
    
    global nn_size_c;
    global w_ih_u;
    rng(5)
    w_ih_x = -1 + 2*rand(nn_size_c, length(x));
    w_ih_u = -1 + 2*rand(nn_size_c, length(u));
    z = w_ih_x*x' + w_ih_u*u;
    out = [((1-exp(-z))./(1+exp(-z)))];

end


function out = actv_func_a(x)

%     b1 = 2.909987461850145;
%     a1 = -b1;
%     
%     b2 = 45;
%     a2 = -b2;
% 
%     xn1 = -1 + ([x(1) x(2)]-a1)*2/(b1-a1);
%     xn2 = -1 + ([x(3) x(4)]-a2)*2/(b2-a2);
%     
%     xn3 = -1 + ([x(5) x(6)]-a1)*2/(b1-a1);
%     xn4 = -1 + ([x(7) x(8)]-a2)*2/(b2-a2);
%     
%     xn = [xn1 xn2 xn3 xn4];

    global nn_size_a;
    rng(1)
    w_ih_x = -1 + 2*rand(nn_size_a, length(x));
    z = [w_ih_x*x'];
    out = [((1-exp(-z))./(1+exp(-z)))];
    
end
  

function phi = kronecker_c(inp_x, inp_u)
   
%     b1 = 2.909987461850145;
%     a1 = -b1;
%     
%     b2 = 45;
%     a2 = -b2;
% 
%     xn1 = -1 + ([inp_x(1) inp_x(2)]-a1)*2/(b1-a1);
%     xn2 = -1 + ([inp_x(3) inp_x(4)]-a2)*2/(b2-a2);
%     
%     xn3 = -1 + ([inp_x(5) inp_x(6)]-a1)*2/(b1-a1);
%     xn4 = -1 + ([inp_x(7) inp_x(8)]-a2)*2/(b2-a2);
%     
%     xn = [xn1 xn2 xn3 xn4];
% 
%     b3 = 150;
%     a3 = -150;
%     
%     un = -1 +  (inp_u - a3)*2/(b3-a3);
%     
%     x = [un' xn xn.^2];
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


function newData = scale(data, max, min, maxRange, minRange)

    newData = minRange + (data-max)*(maxRange-minRange)/(min - max);

end