%% Rede Neural (Linear in the Parameters)

%Função recebe os valores de entradas em um vetor linha
%E recebe a matriz de pesos da rede neural
classdef qFunctionClass
    
    properties
        num_inputs
        num_hiddens
        weights
        gains
    end
    
    methods
        
        function obj = qFunctionClass(num_inputs, num_hiddens, gains)       
            obj.num_inputs = num_inputs;
            obj.num_hiddens = num_hiddens;
            obj.gains = gains;
            obj.weights = obj.initialize_weights();
        end
        
        function weights = initialize_weights(obj)
            if length(obj.gains) < 5
                  weights =zeros(1, obj.num_hiddens);
%                   weights(1) = 1/2;
%                   weights(1+obj.num_inputs) = 1/2;
%                   weights(3) = obj.gains(1); %erro posição elo 1
%                   weights(5) = obj.gains(2); %erro velocidade elo 1
%                   weights(3+obj.num_inputs) =obj.gains(3); %erro posição elo 2
%                   weights(5+obj.num_inputs) = obj.gains(4);%erro velocidade elo 2 
                  weights(1) = 1/2;
                  weights(2) = obj.gains(1);
                  weights(3) = obj.gains(2);
%                   
            else
                weights = obj.gains;
            end
        end
        
        function phi = kronecker(obj, tau, e, x)
            
            z = [tau e e.^2];           
            index = 1;
            for i=1:length(z)
                aux = z(i);
                for j=1:length(z)
                    if j>=i
                        phi(index) = aux*z(j);
                        index = index + 1;
                    end
                end
            end
        end
            
        
        function acao = politica(obj, x, e)           
            % Extraindo do vetor de parametros os pesos associados aos
            % phi*uk do produto de kronecker 
            
            dim_uk = length(e)/2; % Dimensão do vetor uk
            
            w1 = [];
            final = 0;
            inicio = 0;
            
            phi0 = [e e.^2]; % Montando o vetor zk
            
            for i = 0:(dim_uk-1)
                inicio = final + dim_uk + 1 - i;
                final = inicio + length(phi0) - 1;
                w1 = [w1 obj.weights(inicio:final)];
            end 
            
            % Extraindo do vetor de parametros os pesos associados aos
            % uk^2           
            w2 = [];
            salto = 0;
            for i =1:dim_uk
                posicao = i + salto;
                w2 = [w2 obj.weights(posicao)];
                salto = salto + dim_uk + length(phi0) - i;
            end 
            
            %Calculando a ação a ser tomada           
%             phi1 = [phi0 zeros(1,length(phi0))];
%             phi2 = [zeros(1,length(phi0)) phi0];
%             
%             H = (1/2)*inv(diag([w2(1) w2(2)]));            
%             acao = -(H*[phi1; phi2]*w1')';  
%             
            % Descomente para o caso de um link
            H = (1/2)*(1/w2);
            acao = -H*phi0*w1';
                     
        end 
    end
end




