




load('./pixel_files/GraphMap8_2.mat');

fig = imread('./pixel_files/Map8_2.jpg');

SP_list = dlmread('./../distributed/maps/Map_36_SP.txt');

%Transform pixels to meters
resolution = 0.02;

w_s = [0 length(fig(1,:,1)) 0 length(fig(:,1,1))]*resolution;
axis (w_s)

figure(4)
image = (fig(:,:,1)+fig(:,:,2)+fig(:,:,3))/3;
image = flipud(image);
x = linspace(w_s(1),w_s(2),length(fig(1,:,1)));
y = linspace(w_s(3),w_s(4),length(fig(:,1,1)));
[X,Y] = meshgrid(x,y);
H = pcolor(X,Y,image);
H.LineStyle = 'none';
colormap gray
axis equal


load('./output_structure/Original_graph_36.mat','graph')
number_nodes = graph.number_nodes;
node_list = graph.node_list;
edge_matrix = graph.edge_matrix;
complete_edge_matrix = graph.complete_edge_matrix;
complete_SP_matrix = graph.complete_SP_matrix;
map_edge_matrix = graph.map_edge_matrix;
path_matrix = graph.path_matrix;
w_s = graph.w_s;
Pol_coefs = graph.Pol_coefs;
for k = 1:1:length(Pol_coefs)
    
    cx = Pol_coefs(k).coef_x;
    cy = Pol_coefs(k).coef_y;
    %Simulate the computed polynomial
    xsim = [];
    ysim = [];
    dt = 0.05;
    for t = 0:dt:1
        xsim(end+1) = cx(1)*t^5+cx(2)*t^4+cx(3)*t^3+cx(4)*t^2+cx(5)*t^1+cx(6)*t^0;
        ysim(end+1) = cy(1)*t^5+cy(2)*t^4+cy(3)*t^3+cy(4)*t^2+cy(5)*t^1+cy(6)*t^0;
    end
    
    
    %Plot the polynomial
%     figure(4)
    hold on
    plot(xsim,ysim,'b','LineWidth',2)
    hold off

end




hold on
for k = 1:1:length(SP_list(:,1))
    plot(SP_list(k,1),SP_list(k,2),'.r','MarkerSize',25)
    text(SP_list(k,1)+0.1,SP_list(k,2)+0.1,sprintf('%d',k),'FontSize',10,'Color',[1,0,0])
end
hold off
title('Search points indexes')




%Raffle some numbers between 1 and 70
Raffled = [];
count = 0;
while (length(Raffled)<20 && count<100)
    count = count + 1;
    x = rand(1)*77;
    x = ceil(x);
    if (x~=1 && x~=22 && x~=53 && x~=1 && x~=12)
        if length(find(Raffled == x)) == 0
            Raffled(end+1) = x;
        end
    end
end
%<!--SPS = [41 69 6 16 50 25 8 44 34 55 51 74 43 24 57 13 18 17 31 71] -->


%34 -> node where the target will be
%Raffle some groups of four numbers between 1 and 36







load('PosTargets_20.mat','x','y')
hold on
fprintf('Targ_pos = [')
for k = 1:1:length(x)
    plot(x(k),y(k),'.g','MarkerSize',30)
    fprintf('[%.3f, %.3f]', x(k),y(k))
    if k ~= length(x)
        fprintf(', ')
    end
end
fprintf(']\n')
hold off

SPS = [41 69 6 16 50 25 8 44 34 55 51 74 43 24 57 13 18 17 31 71];
fprintf('Past tgis in the world file\n')
for k = 1:1:length(x)
    fprintf('#block( pose [ %.3f %.3f 0 0 ] name "object" color "white") # SP_id: %d\n', x(k),y(k),SPS(k))
end








% dir_content = dir('./../distributed/resultsSP');
% filenames = {dir_content.name};
% for k = 3:1:length(filenames)
%     file = filenames(k);
%     file=num2str(cell2mat(file));
%     file = strcat('./../distributed/resultsSP/',file)
%     R = dlmread(file);
%     size(R)
%     pause()
%    
% end
