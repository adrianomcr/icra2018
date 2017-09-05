


% This script takes the structure of a graph and adds to it a node to every
% edge. The nodes are placed in the midway of the edge


close all;
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




figure(2)
H2 = pcolor(X,Y,image);
title('Virtual graph');
H2.LineStyle = 'none';
colormap gray
axis equal
hold on



number_virtual_nodes = length(Pol_coefs);

hold on
virtual_node_list = [];
for k = 1:1:number_virtual_nodes
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
    
    figure(1)
    hold on
%     plot(virtual_node_list(k,1),virtual_node_list(k,2),'*b','LineWidth',2)
%     plot(virtual_node_list(k,1),virtual_node_list(k,2),'ob','LineWidth',2)
%     text(virtual_node_list(k,1)+0.05,virtual_node_list(k,2)+0.05,sprintf('%d',k),'FontSize',10,'Color',[0 0 1])

    %Plot the cost of the edges
%     text(virtual_node_list(k,1)+0.05,virtual_node_list(k,2)+0.05,sprintf('%.3f',edge_matrix(i,j)),'FontSize',10,'Color',[0 0 0])
    
    
end
hold off




%%
figure(2)
%creating the new edge matrix
E = zeros(length(Pol_coefs),length(Pol_coefs));
% Ed_virtual_matrix = [];
n_edges = 0;
for k = 1:1:number_virtual_nodes
    i = Pol_coefs(k).from;
    j = Pol_coefs(k).to;
    

    %Finding neigbohrs in the "from node" side
    neig = edge_matrix(i,:);
    for m = 1:1:length(neig)
        if(neig(m) ~= 0 && m ~= j) %Then there is an edge from virtual node k to every virtual node whose equivaent edge is conected to node m
            k2 = map_edge_matrix(i,m);
            if(k > k2)
                %E(k,k2) = E(k,k2)+1;
                E(k,k2) = edge_matrix(i,j)/2+edge_matrix(i,m)/2;
                n_edges = n_edges + 1;
                
%                 Ed_virtual_matrix(n_edges) = struct('from',k,'to',k2,'coef_x',[],'coef_y',[],'cost',E(k,k2));
                
                %Compute polynomial
                %x from 0.5 of k to 0or1 of k
                %x from 0or1 of k2 to 0.5 of k2
                cx = Pol_coefs(k).coef_x;
                cy = Pol_coefs(k).coef_y;
                x = [];
                y = [];
                if(graph.Pol_coefs(k).from == i)
                    p1 = 0.5:-0.01:0;
                elseif(graph.Pol_coefs(k).to == i)
                    p1 = 0.5:0.01:1;
                end
                for p = p1
                    x(end+1) = cx(1)*p^5+cx(2)*p^4+cx(3)*p^3+cx(4)*p^2+cx(5)*p^1+cx(6)*p^0;
                    y(end+1) = cy(1)*p^5+cy(2)*p^4+cy(3)*p^3+cy(4)*p^2+cy(5)*p^1+cy(6)*p^0;
                end
                
%                 figure(1)
%                 hold on
%                 plot(x,y,'y','LineWidth',4);
%                 pause();
%                 figure(2)
                
                cx = Pol_coefs(k2).coef_x;
                cy = Pol_coefs(k2).coef_y;
                if(graph.Pol_coefs(k2).from == i)
                    p1 = 0:0.01:0.5;
                elseif(graph.Pol_coefs(k2).to == i)
                    p1 = 1:-0.01:0.5;
                end
                for p = p1
                    x(end+1) = cx(1)*p^5+cx(2)*p^4+cx(3)*p^3+cx(4)*p^2+cx(5)*p^1+cx(6)*p^0;
                    y(end+1) = cy(1)*p^5+cy(2)*p^4+cy(3)*p^3+cy(4)*p^2+cy(5)*p^1+cy(6)*p^0;
                end
                
                A = [];
                for p = linspace(0,1,length(x))
                    A = [A; p^5 p^4 p^3 p^2 p^1 p^0];
                end
                
                Ed_virtual_matrix(n_edges) = struct('from',k,'to',k2,'coef_x',A\x','coef_y',A\y','cost',E(k,k2));
                
%                 figure(1)
%                 hold on
%                 plot(x(1:90),y(1:90),'g','LineWidth',1);
%                 pause();
%                 figure(2)
                
            end
        end
    end

    %Finding neigbohrs in the "to node" side
    neig = edge_matrix(j,:);
    for m = 1:1:length(neig)
        if(neig(m) ~= 0 && m ~= i) %Then there is an edge from virtual node k to every virtual node whose equivaent edge is conected to node m
            k2 = map_edge_matrix(j,m);
            if(k > k2)
                %E(k,k2) = E(k,k2)+1;
                E(k,k2) = edge_matrix(i,j)/2+edge_matrix(j,m)/2;
                n_edges = n_edges + 1;
                Ed_virtual_matrix(n_edges) = struct('from',k,'to',k2,'coef_x',[],'coef_y',[],'cost',E(k,k2));
                %Compute polynomial
%                 x from 0.5 of k to 1 of k
%                 x from 0 of k2 to 0.5 of k2


                %Compute polynomial
                %x from 0.5 of k to 0or1 of k
                %x from 0or1 of k2 to 0.5 of k2
                cx = Pol_coefs(k).coef_x;
                cy = Pol_coefs(k).coef_y;
                x = [];
                y = [];
                if(graph.Pol_coefs(k).from == j)
                    p1 = 0.5:-0.01:0;
                elseif(graph.Pol_coefs(k).to == j)
                    p1 = 0.5:0.01:1;
                end
                for p = p1
                    x(end+1) = cx(1)*p^5+cx(2)*p^4+cx(3)*p^3+cx(4)*p^2+cx(5)*p^1+cx(6)*p^0;
                    y(end+1) = cy(1)*p^5+cy(2)*p^4+cy(3)*p^3+cy(4)*p^2+cy(5)*p^1+cy(6)*p^0;
                end
                
%                 figure(1)
%                 hold on
%                 plot(x,y,'y','LineWidth',4);
%                 pause();
%                 figure(2)
                
                cx = Pol_coefs(k2).coef_x;
                cy = Pol_coefs(k2).coef_y;
                if(graph.Pol_coefs(k2).from == j)
                    p1 = 0:0.01:0.5;
                elseif(graph.Pol_coefs(k2).to == j)
                    p1 = 1:-0.01:0.5;
                end
                for p = p1
                    x(end+1) = cx(1)*p^5+cx(2)*p^4+cx(3)*p^3+cx(4)*p^2+cx(5)*p^1+cx(6)*p^0;
                    y(end+1) = cy(1)*p^5+cy(2)*p^4+cy(3)*p^3+cy(4)*p^2+cy(5)*p^1+cy(6)*p^0;
                end
                
                A = [];
                for p = linspace(0,1,length(x))
                    A = [A; p^5 p^4 p^3 p^2 p^1 p^0];
                end
                
                Ed_virtual_matrix(n_edges) = struct('from',k,'to',k2,'coef_x',A\x','coef_y',A\y','cost',E(k,k2));
                
%                 figure(1)
%                 hold on
%                 plot(x(1:90),y(1:90),'g','LineWidth',1);
%                 pause();
%                 figure(2)





                
            end
        end
    end
    
    
end
E = E+E';

% figure(1)
hold on
for i = 1:1:number_virtual_nodes
    for j = 1:1:length(virtual_node_list)
        if(i>j && E(i,j) ~= 0)
            n1 = virtual_node_list(i,:);
            n2 = virtual_node_list(j,:);
%             plot([n1(1) n2(1)],[n1(2) n2(2)],'--r','LineWidth',1)

            %Plot the cost of the edges
%             text((n1(1)+n2(1))/2+0.05,(n1(2)+n2(2))/2+0.05,sprintf('%.3f',E(i,j)),'FontSize',10,'Color',[0 0 0])
        end
    end
end
for k = 1:1:n_edges
    cx = Ed_virtual_matrix(k).coef_x;
    cy = Ed_virtual_matrix(k).coef_y;
    x = []; y = [];
    for p = 0:0.01:1
        x(end+1) = cx(1)*p^5+cx(2)*p^4+cx(3)*p^3+cx(4)*p^2+cx(5)*p^1+cx(6)*p^0;
        y(end+1) = cy(1)*p^5+cy(2)*p^4+cy(3)*p^3+cy(4)*p^2+cy(5)*p^1+cy(6)*p^0;
    end
    plot(x,y,'r-','LineWidth',1)
end
for k = 1:1:number_virtual_nodes
    plot(virtual_node_list(k,1),virtual_node_list(k,2),'*b','LineWidth',2)
    plot(virtual_node_list(k,1),virtual_node_list(k,2),'ob','LineWidth',2)
    text(virtual_node_list(k,1)+0.05,virtual_node_list(k,2)+0.05,sprintf('%d',k),'FontSize',10,'Color',[0 0 1])
end
hold off






%Creating complete virtual graph


% Create container
G = container_set(vertex.empty());

path = struct('path',[]);

load('./output_structure/Original_graph_36.mat','graph')
for i = 1:1:number_virtual_nodes
    x = virtual_node_list(i,:);
    neig = [];
    cost = [];
    for j = 1:1:number_virtual_nodes
        if(j ~= i)
            if(E(i,j) ~= 0)
                neig = [neig j];
                cost = [cost E(i,j)];
            end
        end
    end
    v = vertex(G.get_next_idx(), x, 0, cost, 0, neig, [], 0);
    G.add_element(v); 
end
complete_edge_matrix_virtual = zeros(number_virtual_nodes,number_virtual_nodes);
for i = 1:1:number_virtual_nodes
    [success, CC, EE] = search_Dijkstra(i, 0, G);
    for j = 1:1:number_virtual_nodes
        if (i ~= j)
            p = path;
            p.path = CC.container(j).traj_from_start;
            Paths(i,j) = p;
            complete_edge_matrix_virtual(i,j) = CC.container(j).cost_from_start;
        end
    end
end




%Creating a matrix tha maps a virtual node pair (i, j) to a index ithe edge list
map_edge_matrix_virtual = -1*ones(number_virtual_nodes,number_virtual_nodes);
for k = 1:1:length(Ed_virtual_matrix)
    map_edge_matrix_virtual(Ed_virtual_matrix(k).from,Ed_virtual_matrix(k).to) = k;
    map_edge_matrix_virtual(Ed_virtual_matrix(k).to,Ed_virtual_matrix(k).from) = k;
end




graph3.number_nodes = number_virtual_nodes;
graph3.node_list = virtual_node_list;
graph3.edge_matrix = E;
graph3.complete_edge_matrix = complete_edge_matrix_virtual;
graph3.map_edge_matrix = map_edge_matrix_virtual;
graph3.path_matrix = Paths;
graph3.w_s = w_s;
graph3.Pol_coefs = Ed_virtual_matrix;

graph = graph3;
save('./output_structure/Virtual_graph_36.mat','graph')





