

% close all;
clear all; clc;

%Adds the path to the graph library
addpath('./graphutils')

fig = imread('./pixel_files/Map8_2.jpg');
load('./output_structure/Original_graph_36.mat','graph')
number_nodes = graph.number_nodes;
node_list = graph.node_list;
edge_matrix = graph.edge_matrix;
complete_edge_matrix = graph.complete_edge_matrix;
map_edge_matrix = graph.map_edge_matrix;
path_matrix = graph.path_matrix;
w_s = graph.w_s;
Pol_coefs = graph.Pol_coefs;

%Plotting the original graph
figure(1)
image = (fig(:,:,1)+fig(:,:,2)+fig(:,:,3))/3;
image = flipud(image);
x = linspace(w_s(1),w_s(2),length(fig(1,:,1)));
y = linspace(w_s(3),w_s(4),length(fig(:,1,1)));
[X,Y] = meshgrid(x,y);
H = pcolor(X,Y,image);
title('Original graph');
H.LineStyle = 'none';
colormap gray
axis equal
hold on
for k = 1:1:length(Pol_coefs)
    cx = Pol_coefs(k).coef_x;
    cy = Pol_coefs(k).coef_y;
    xp = [];
    yp = [];
    for t = 0:0.01:1
        xp(end+1) = cx(1)*t^5+cx(2)*t^4+cx(3)*t^3+cx(4)*t^2+cx(5)*t^1+cx(6)*t^0;
        yp(end+1) = cy(1)*t^5+cy(2)*t^4+cy(3)*t^3+cy(4)*t^2+cy(5)*t^1+cy(6)*t^0;
    end
    plot(xp,yp,'b','LineWidth',2)
end
plot(node_list(:,1),node_list(:,2),'*r','LineWidth',2)
plot(node_list(:,1),node_list(:,2),'or','LineWidth',2)
for k = 1:1:length(node_list)
    text(node_list(k,1)+0.05,node_list(k,2)+0.05,sprintf('%d',k),'FontSize',10,'Color',[1 0 0])
end
hold off



% ----------  ----------  ----------  ----------  ----------  ----------
% ----------  ----------  ----------  ----------  ----------  ----------
MAX_DIST = 2.0;
SP = struct('SP_number',[],'parameter_values',[]);
search_point_list = [SP];
for k = 1:1:length(Pol_coefs)
    n_SP = ceil(Pol_coefs(k).cost/MAX_DIST); % Number of searching points in the edge
    search_point_list(k) = SP;
    search_point_list(k).SP_number = n_SP;
    %if (n_SP ~= 0)
        d_p = 1/n_SP;
        for l = 1:1:n_SP
            search_point_list(k).parameter_values(end+1) = (l-1)*d_p;
        end
    %end
end


figure(1)
hold on
for k = 1:1:length(Pol_coefs)
    cx = Pol_coefs(k).coef_x;
    cy = Pol_coefs(k).coef_y;
    for p = search_point_list(k).parameter_values
        xp(end+1) = cx(1)*p^5+cx(2)*p^4+cx(3)*p^3+cx(4)*p^2+cx(5)*p^1+cx(6)*p^0;
        yp(end+1) = cy(1)*p^5+cy(2)*p^4+cy(3)*p^3+cy(4)*p^2+cy(5)*p^1+cy(6)*p^0;
    end
end
hold off