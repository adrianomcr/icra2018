


% This script takes the structure of a graph and adds to it a node to every
% edge. The nodes are placed in the midway of the edge


close all;
clear all; clc;

%Adds the path to the graph library
addpath('./graphutils')

fig = imread('./pixel_files/Map8_2.jpg');
load('./output_structure/Graph_data_36_meters.mat','graph')
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

hold on
virtual_node_list = [];
for k = 1:1:length(Pol_coefs)
    i = Pol_coefs(k).from;
    j = Pol_coefs(k).to;
    cx = Pol_coefs(k).coef_x;
    cy = Pol_coefs(k).coef_y;
    L = edge_matrix(i,j);
    
    %Itegrate the edge until fing the midpoint
    dp = 10e-4;
    comp = 0;
    p1 = [cx(6); cy(6)];
    for p = dp:dp:1
        p0 = p1;
        p1 = [cx(1)*p^5+cx(2)*p^4+cx(3)*p^3+cx(4)*p^2+cx(5)*p^1+cx(6)*p^0; cy(1)*p^5+cy(2)*p^4+cy(3)*p^3+cy(4)*p^2+cy(5)*p^1+cy(6)*p^0];
        comp = comp + sqrt((p0-p1)'*(p0-p1));
        if(comp >= L/2)
            break
        end
    end
    
%     node_list(end+1,:) = (p0'+p1')/2;
    virtual_node_list(end+1,:) = (p0'+p1')/2;
    
    plot(virtual_node_list(end,1),virtual_node_list(end,2),'*g','LineWidth',2)
    plot(virtual_node_list(end,1),virtual_node_list(end,2),'og','LineWidth',2)
    text(virtual_node_list(end,1)+0.05,virtual_node_list(end,2)+0.05,sprintf('%d',length(node_list(:,1))+k),'FontSize',10,'Color',[0 1 0])
    
    
end
hold off

%Updating the cost matrix
edge_matrix(end+length(Pol_coefs),end+length(Pol_coefs)) = 0;
for k = 1:1:length(Pol_coefs)
    i = Pol_coefs(k).from;
    j = Pol_coefs(k).to;
    cx = Pol_coefs(k).coef_x;
    cy = Pol_coefs(k).coef_y;
    L = edge_matrix(i,j);
    
    %Index of the virtual node
    v = length(node_list(:,1)) + k;
    
    %Exclude the direct connection between i and j
    edge_matrix(i,j) = 0;
    edge_matrix(j,i) = 0;
    
    %Create a new connection between i and v and between j and v
    edge_matrix(i,v) = L/2;
    edge_matrix(v,i) = L/2;
    edge_matrix(j,v) = L/2;
    edge_matrix(v,j) = L/2;
    
end





% Create container
G = container_set(vertex.empty());

number_nodes = number_nodes + length(virtual_node_list(:,1));
node_list = [node_list; virtual_node_list];

path = struct('path',[]);

for i = 1:1:number_nodes
    x = node_list(i,:);
    neig = [];
    cost = [];
    for j = 1:1:number_nodes
        if(j ~= i)
            if(edge_matrix(i,j) ~= 0)
                neig = [neig j];
                cost = [cost edge_matrix(i,j)];
            end
        end
    end
    v = vertex(G.get_next_idx(), x, 0, cost, 0, neig, [], 0);
    G.add_element(v); 
end
complete_edge_matrix = zeros(number_nodes,number_nodes);
for i = 1:1:number_nodes
    [success, CC, EE] = search_Dijkstra(i, 0, G);
    for j = 1:1:number_nodes
        if (i ~= j)
            p = path;
            p.path = CC.container(j).traj_from_start;
            Paths(i,j) = p;
            complete_edge_matrix(i,j) = CC.container(j).cost_from_start;
        end
    end
end







%Creating new polynomials
for k = 1:1:length(Pol_coefs)
    i = graph.Pol_coefs(k).from;
    j = graph.Pol_coefs(k).to;
    cx = graph.Pol_coefs(k).coef_x;
    cy = graph.Pol_coefs(k).coef_y;
    
    v = graph.number_nodes + k;
    
    p = linspace(0,0.5,25);
    A = []; x = []; y = [];
    for ip = 1:1:25
        x(end+1) = cx(1)*p(ip)^5+cx(2)*p(ip)^4+cx(3)*p(ip)^3+cx(4)*p(ip)^2+cx(5)*p(ip)^1+cx(6)*p(ip)^0;
        y(end+1) = cy(1)*p(ip)^5+cy(2)*p(ip)^4+cy(3)*p(ip)^3+cy(4)*p(ip)^2+cy(5)*p(ip)^1+cy(6)*p(ip)^0;
    end
    p = linspace(0,1,25);
    for ip = 1:1:25
        A = [A; p(ip)^5 p(ip)^4 p(ip)^3 p(ip)^2 p(ip)^1 p(ip)^0];
    end
    
    cx1 = A\x';
    cy1 = A\y';
    
    %Create a "structure for a polynomial" and store it in the matrix
    pol = struct('from',i,'to',v,'coef_x',cx1,'coef_y',cy1);
    Pol_coefs(2*k-1) = pol;
    

    p = linspace(0.5,1,25);
    A = []; x = []; y = [];
    for ip = 1:1:25
        x(end+1) = cx(1)*p(ip)^5+cx(2)*p(ip)^4+cx(3)*p(ip)^3+cx(4)*p(ip)^2+cx(5)*p(ip)^1+cx(6)*p(ip)^0;
        y(end+1) = cy(1)*p(ip)^5+cy(2)*p(ip)^4+cy(3)*p(ip)^3+cy(4)*p(ip)^2+cy(5)*p(ip)^1+cy(6)*p(ip)^0;
    end
    p = linspace(0,1,25);
    for ip = 1:1:25
        A = [A; p(ip)^5 p(ip)^4 p(ip)^3 p(ip)^2 p(ip)^1 p(ip)^0];
    end
    
    cx2 = A\x';
    cy2 = A\y';
    
    %Create a "structure for a polynomial" and store it in the matrix
    pol = struct('from',v,'to',j,'coef_x',cx2,'coef_y',cy2);
    Pol_coefs(2*k) = pol;
    


end




map_edge_matrix = -1*ones(number_nodes,number_nodes);
for k = 1:1:length(Pol_coefs)
    map_edge_matrix(Pol_coefs(k).from,Pol_coefs(k).to) = k;
    map_edge_matrix(Pol_coefs(k).to,Pol_coefs(k).from) = k;
end




graph2.number_nodes = number_nodes;
graph2.node_list = node_list;
graph2.edge_matrix = edge_matrix;
graph2.complete_edge_matrix = complete_edge_matrix;
graph2.map_edge_matrix = map_edge_matrix;
graph2.path_matrix = Paths;
graph2.w_s = w_s;
graph2.Pol_coefs = Pol_coefs;

 
save('./output_structure/Graph_data_36_meters_virtuals.mat','graph2')






%%

% 
% %Plotting the graph with the partioned polynomials
% figure(2)
% x = linspace(w_s(1),w_s(2),length(fig(1,:,1)));
% y = linspace(w_s(3),w_s(4),length(fig(:,1,1)));
% [X,Y] = meshgrid(x,y);
% H = pcolor(X,Y,image);
% H.LineStyle = 'none';
% colormap gray
% axis equal
% hold on
% for k = 1:1:length(Pol_coefs)
%     cx = Pol_coefs(k).coef_x;
%     cy = Pol_coefs(k).coef_y;
%     xp = [];
%     yp = [];
%     for t = 0:0.01:1
%         xp(end+1) = cx(1)*t^5+cx(2)*t^4+cx(3)*t^3+cx(4)*t^2+cx(5)*t^1+cx(6)*t^0;
%         yp(end+1) = cy(1)*t^5+cy(2)*t^4+cy(3)*t^3+cy(4)*t^2+cy(5)*t^1+cy(6)*t^0;
%     end
% %     plot(xp,yp,'b','LineWidth',2)
%     if(mod(k,2) == 1)
%         plot(xp,yp,'b','LineWidth',2)
%     else
%         plot(xp,yp,'c','LineWidth',2)
%     end
% end
% % plot(node_list(:,1),node_list(:,2),'*r','LineWidth',2)
% % plot(node_list(:,1),node_list(:,2),'or','LineWidth',2)
% plot(node_list(1:36,1),node_list(1:36,2),'*r','LineWidth',2)
% plot(node_list(1:36,1),node_list(1:36,2),'or','LineWidth',2)
% plot(node_list(37:end,1),node_list(37:end,2),'*g','LineWidth',2)
% plot(node_list(37:end,1),node_list(37:end,2),'og','LineWidth',2)
% for k = 1:1:length(node_list)
% %     text(node_list(k,1)+0.05,node_list(k,2)+0.05,sprintf('%d',k),'FontSize',10,'Color',[1 0 0])
%     if(k<36)
%         text(node_list(k,1)+0.05,node_list(k,2)+0.05,sprintf('%d',k),'FontSize',10,'Color',[1 0 0])
%     else
%         text(node_list(k,1)+0.05,node_list(k,2)+0.05,sprintf('%d',k),'FontSize',10,'Color',[0 1 0])
%     end
% end
% hold off
