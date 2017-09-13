
close all;
clear all; clc;


load('./pixel_files/GraphMap8_2.mat');

fig = imread('./pixel_files/Map8_2.jpg');



%Transform pixels to meters
resolution = 0.02;

w_s = [0 length(fig(1,:,1)) 0 length(fig(:,1,1))]*resolution;
axis (w_s)

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


%Adds the path to the graph library
addpath('./graphutils')

graph = struct('number_nodes',[],'node_list',[],'edge_matrix',[],'complete_edge_matrix',[],'map_edge_matrix',[],'path_matrix',[],'w_s',[],'Pol_coefs',[]);

graph.w_s = w_s;
graph.number_nodes = 36;


%Get the position of the nodes (vertices)
nodes = inf*ones(36,2);
for k = 1:1:length(Path_l)
    node_id = Path_l(k).from;
    nodes(node_id,1) = Path_l(k).path(1,2)*resolution;
    nodes(node_id,2) = w_s(4)-Path_l(k).path(1,1)*resolution;
    node_id = Path_l(k).To;
    nodes(node_id,1) = Path_l(k).path(end,2)*resolution;
    nodes(node_id,2) = w_s(4)-Path_l(k).path(end,1)*resolution;
end


graph.node_list = nodes;
% graph.edge_matrix = ComG;


% save('Graph_data_36_0','graph')


% %Plotting nodes
% hold on
% for k = 1:1:length(nodes(:,1))
%     plot(nodes(k,1),nodes(k,2),'*r','LineWidth',2)
%     plot(nodes(k,1),nodes(k,2),'or','LineWidth',2)
% end
% hold off





path = struct('path',[]);
E = [];
paths = path;
k = 0;
for i = 1:1:graph.number_nodes
    for j = 1:1:graph.number_nodes
        if (i ~= j)
            k = k+1;% for k = 1:1:length(E(:,1))
            E(k,:) = [i, j];
            paths(k,1) = path;
        end
    end
end






% graph.edge_matrix = graph.edge_matrix*resolution;

%Matrix of lengths of the edges
E_pol = zeros(length(MeetPoints));


%Loop to generate a set of polynomials that represents each edge
for k = 1:1:length(Path_l)
    i = Path_l(k).from;
    j = Path_l(k).To;
    x = Path_l(k).path;
    y = w_s(4)-x(:,1)*resolution;
    x = x(:,2)*resolution;

%     figure(2)
%     hold on
%     plot(x,y,'y+','LineWidth',1)
%     axis equal


    %Create matrix of regressors
    A = [];
    t = 0;
    dt = 1/length(x);
    for p = 1:1:length(x)
        A = [A; t^5 t^4 t^3 t^2 t^1 t^0];
        t = t+dt;
    end

    %Apply the minimum squares method
    cx = A\x;
    cy = A\y;
    
%     %Create a "structure for a polynomial" and store it in the matrix
%     pol = struct('from',Path_l(k).from,'to',Path_l(k).To,'coef_x',cx,'coef_y',cy,'cost',[]);
%     Pol_coefs(k) = pol;

    %Simulate the computed polynomial
    xsim = [];
    ysim = [];
    for t = 0:dt:1
        xsim(end+1) = cx(1)*t^5+cx(2)*t^4+cx(3)*t^3+cx(4)*t^2+cx(5)*t^1+cx(6)*t^0;
        ysim(end+1) = cy(1)*t^5+cy(2)*t^4+cy(3)*t^3+cy(4)*t^2+cy(5)*t^1+cy(6)*t^0;
    end
    %Compute the length of a edge based on the computed polinomial
    comp = 0;
    for p = 1:1:(length(xsim)-1)
        d_comp = sqrt((xsim(p)-xsim(p+1))^2+(ysim(p)-ysim(p+1))^2);
        comp = comp + d_comp;
    end
    E_pol(Path_l(k).from,Path_l(k).To) = comp;
    
    %Create a "structure for a polynomial" and store it in the matrix
    pol = struct('from',Path_l(k).from,'to',Path_l(k).To,'coef_x',cx,'coef_y',cy,'cost',comp);
    Pol_coefs(k) = pol;

    
    %Plot the polynomial
%     figure(1)
    hold on
    plot(xsim,ysim,'b','LineWidth',2)
    hold off
%     figure(2)
%     hold on
%     plot(xsim,ysim,'b','LineWidth',2)
%     hold off
end
E_pol = E_pol + E_pol';






% Create container
G = container_set(vertex.empty());


for i = 1:1:graph.number_nodes
    x = graph.node_list(i,:);
    neig = [];
    cost = [];
    for j = 1:1:graph.number_nodes
        if(j ~= i)
            if(E_pol(i,j) ~= 0)
                neig = [neig j];
                cost = [cost E_pol(i,j)];
            end
        end
    end
    v = vertex(G.get_next_idx(), x, 0, cost, 0, neig, [], 0);
    G.add_element(v); 
end



complete_edge_matrix = zeros(graph.number_nodes,graph.number_nodes);
for i = 1:1:graph.number_nodes
    [success, CC, EE] = search_Dijkstra(i, 0, G);
    for j = 1:1:graph.number_nodes
        if (i ~= j)
            p = path;
            p.path = CC.container(j).traj_from_start;
            Paths(i,j) = p;
            complete_edge_matrix(i,j) = CC.container(j).cost_from_start;
        end
    end
end
graph.complete_edge_matrix = complete_edge_matrix;



map_edge_matrix = -1*ones(graph.number_nodes,graph.number_nodes);
for k = 1:1:length(Pol_coefs)
    map_edge_matrix(Pol_coefs(k).from,Pol_coefs(k).to) = k;
    map_edge_matrix(Pol_coefs(k).to,Pol_coefs(k).from) = k;
end
graph.map_edge_matrix = map_edge_matrix;


graph.path_matrix = Paths;
graph.edge_matrix = E_pol;


%Plotting nodes
hold on
for k = 1:1:length(nodes(:,1))
    plot(nodes(k,1),nodes(k,2),'*r','LineWidth',2)
    plot(nodes(k,1),nodes(k,2),'or','LineWidth',2)
    text(nodes(k,1)+0.1,nodes(k,2)+0.1,sprintf('%d',k),'FontSize',15,'color',[0 1 0])
end
hold off



% Add the polinomials to the structure
graph.Pol_coefs = Pol_coefs;

axis(w_s)

% Save structure
save('./output_structure/Original_graph_36.mat','graph')



