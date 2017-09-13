


CreateOnlyVirtualNodes

M = dlmread('./../distributed/maps/Map_36_SP.txt');



figure(1)
hold on
plot(M(:,1),M(:,2),'.g','MarkerSize',20)
hold off

figure(2)
hold on
plot(M(:,1),M(:,2),'.g','MarkerSize',20)
hold off
%for k = 1:1:length(M(1,:))
%    plot()
%end

%%

load('output_structure/Original_graph_36.mat','graph')
%In he original graph
% N_SP = [1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40]
N_SP = [2, 7, 4, 2, 2, 4, 3, 3, 4, 2, 2, 3, 2, 5, 5, 5, 3, 3, 3, 2, 3, 2, 3, 3, 2, 3, 2, 2, 5, 4, 3, 4, 3, 1, 2, 2, 2, 3, 3, 2];
C_sp = zeros(graph.number_nodes,graph.number_nodes);
for i = 1:1:graph.number_nodes
    for j = 1:1:graph.number_nodes
        if (i ~= j)
            p = graph.path_matrix(i,j);
            e = graph.map_edge_matrix(p.path(end-1),p.path(end));
            C_sp(i,j) = N_SP(e);
        end
    end
end



% %In he virtual graph
% % N_SP = [1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66]
% N_SP = [];
