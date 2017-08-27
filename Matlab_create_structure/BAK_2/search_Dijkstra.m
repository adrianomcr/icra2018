%Universidade Federal de Minas Gerais - 2016/2
%Planejamento de Movimento de Robos II
%Aluno: Adriano Martins da Costa Rezende
%Professor: Guilherme Pereira

function [success, CC, EE] = search_Dijkstra(v, f, SS)

    %v - noh inicial
    %f - noh final - objetivo
    %S - especificacao do grafo
    %OBS: no campo children_set de 'S' nao estao os filhos, mas sim os vizinhos!!
    
    S = SS;
    
    success = 0; %inifialmente f ainda nao foi encontrado
    
    %container_set onde a arvore sera construida
    CC = container_set(vertex.empty());
    %grupo de vertices da arvore a ser criada
    EE = edge_set();
    
    
    
    S.container(v).children_set = [S.container(v).children_set v];
    S.container(v).cost_from_parent = [S.container(v).cost_from_parent 0];
    
    
    % coloca v em O
    O=[v v 0]; %[noh, noh pai, custo]
    % Inicia o conjunto fechado C
    C=[];
     
    % enquanto a fila esta vazia
    while length(O(:,1)) > 0
        % remove u de O
        u=O(1,1);
        pu=O(1,2); % parent of u
        cu=O(1,3); % cost of u
        O(1,:)=[];
        
        % Se u nao estiver em C coloque u em C
        if length(find(C==u))==0
            % coloque u em C
            C(end+1)=u;
            n = S.container(u);
            n.parent_idx=pu;
%             n.cost_from_parent = pdist2(S.container(pu).state,n.state,'euclidean');
            n.cost_from_parent = S.container(u).cost_from_parent(find(n.children_set==pu));
% S.container(u).cost_from_parent(find(S.container(u).children_set==pu))
% [u, pu, ...
% pdist2(S.container(pu).state,n.state,'euclidean'), ...
% S.container(u).cost_from_parent(find(n.children_set==pu))]
            CC.add_element(n);
            CC.container(n.idx).traj_from_start = [];
            CC.container(u).cost_from_start = CC.container(pu).cost_from_start + n.cost_from_parent;
            CC.container(u).traj_from_start = [CC.container(pu).traj_from_start u];
            EE.add_edge(u, pu);

            %Esvazie a lista (antiga) de filhos do noh
            CC.container(u).children_set = [];
            
            if (u ~= v) %Se o noh nao eh a raiz
                %Coloque o noh na lista de filhos pertencente ao seu noh pai
                CC.container(pu).children_set(end+1) = u;
            end
            
%             %Retorna se o noh objetivo for encontrado
%             if (C(end) == f)
%                 success = 1; %indica sucesso
%                 return
%             end
            
            % Coloque os vizinhos do u que nao estao em C em O
            for i=1:length(S.container(u).children_set)
                if length(find(C==S.container(u).children_set(i)))==0
                    O(end+1,1)=S.container(u).children_set(i);
                    O(end,2)=u;

%                     u
%                     S
%                     S.container(u).cost_from_parent(1);
%                     O(end,3)=CC.container(u).cost_from_start + pdist2(S.container(u).state,S.container(O(end,1)).state,'euclidean');
%                     O(end,3)=CC.container(u).cost_from_start + S.container(u).cost_from_parent(find(S.container(u).children_set==O(end,1)));

% 'a'
% CC.container(u).cost_from_start
% 'b'
% aa = find(S.container(u).children_set==O(end,1))
% 'c'
% S.container(u).cost_from_parent(aa)
% 'd'
% S.container(u).cost_from_parent(find(S.container(u).children_set==O(end,1)))
% 'e'
% O(end,3)
% 'f'
                    O(end,3)=CC.container(u).cost_from_start + S.container(u).cost_from_parent(find(S.container(u).children_set==O(end,1)));
                    
                    
%                     O(end,3)=CC.container(u).cost_from_start + pdist2(S.container(u).state,S.container(O(end,1)).state,'euclidean');
                    
                end
            end
            
            %Ordena os dados em O de acordo com o custo (terceira coluna)
            [O(:,3), order] = sort(O(:,3));
            O(:,1) = O(order,1);
            O(:,2) = O(order,2);

                     
        end 
        
    end

end