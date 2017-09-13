


%This script builds a list of tuples to be solved by the
%Chinese-Postman-Problem




load('./output_structure/Original_graph_36.mat','graph')


M = [];
for k = 1:1:length(graph.Pol_coefs)
    M(k,1) = graph.Pol_coefs(k).from;
    M(k,2) = graph.Pol_coefs(k).to;
    M(k,3) = graph.edge_matrix(M(k,1),M(k,2));
end


fprintf('list_edges = [')
for k = 1:1:(length(M(:,1))-1)
    fprintf('(%d, %d, %.4f),',M(k,1),M(k,2),M(k,3))
end
fprintf('(%d, %d, %.4f)]\n',M(end,1),M(end,2),M(end,3))

