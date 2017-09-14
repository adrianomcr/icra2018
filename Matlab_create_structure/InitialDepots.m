%Initial_depots

SP_list = dlmread('./../distributed/maps/Map_36_SP.txt');

load('output_structure/Original_graph_36.mat','graph');
node_list = graph.node_list;

% 
% count = 0;
% for R = 1:1:4
% target_node = 20; %SP_45
% Raffled_depots = zeros(20,R);
% for k = 1:1:20
%     count = count + 1;
%     x = [];
%     while length(x)<R
%         x2 = rand(1)*36;
%         x2 = ceil(x2);
%         if x2 ~= target_node
%             if length(find(x == x2))==0
%                 if length(x)>0
%                     d_min = norm(node_list(target_node,:)-node_list(x2,:),2);
%                     for k2 = 1:1:length(x)
%                         d = norm(node_list(x(k2),:)-node_list(x2,:),2);
%                         if d<d_min; d_min = d; end
%                     end
%                     if d_min > 3.5
%                         x(end+1) = x2;
%                     end
%                 else
%                     if(norm(node_list(target_node,:)-node_list(x2,:),2) < 3.5)
%                         x(end+1) = x2;
%                     end
%                 end
%             end
%         end
%     end
%     Raffled_depots(k,:) = x;
% 
% end
% Raffled_depots
% name = sprintf('Raffled_depots_%d.mat',R);
% save(name,'Raffled_depots')
% end




R = 4;
switch (R)
    case 1
        load('Raffled_depots_1.mat','Raffled_depots')
    case 2  
        load('Raffled_depots_2.mat','Raffled_depots')
    case 3
        load('Raffled_depots_3.mat','Raffled_depots')
    case 4
        load('Raffled_depots_4.mat','Raffled_depots')
end

% depot = 4;
clc
for depot = 1:1:20;
fprintf('#Set %d\n',depot)
switch (R)
    case 1
        fprintf('#erratic( pose [%.3f %.3f 0 0] name "robot_0" color "blue") # node %d\n',node_list(Raffled_depots(depot,1),1),node_list(Raffled_depots(depot,1),2),Raffled_depots(depot,1))
    case 2
        fprintf('#erratic( pose [%.3f %.3f 0 0] name "robot_0" color "blue") # node %d\n',node_list(Raffled_depots(depot,1),1),node_list(Raffled_depots(depot,1),2),Raffled_depots(depot,1))
        fprintf('#erratic( pose [%.3f %.3f 0 0] name "robot_1" color "red") # node %d\n',node_list(Raffled_depots(depot,2),1),node_list(Raffled_depots(depot,2),2),Raffled_depots(depot,2))
    case 3
        fprintf('#erratic( pose [%.3f %.3f 0 0] name "robot_0" color "blue") # node %d\n',node_list(Raffled_depots(depot,1),1),node_list(Raffled_depots(depot,1),2),Raffled_depots(depot,1))
        fprintf('#erratic( pose [%.3f %.3f 0 0] name "robot_1" color "red") # node %d\n',node_list(Raffled_depots(depot,2),1),node_list(Raffled_depots(depot,2),2),Raffled_depots(depot,2))
        fprintf('#erratic( pose [%.3f %.3f 0 0] name "robot_2" color "green") # node %d\n',node_list(Raffled_depots(depot,3),1),node_list(Raffled_depots(depot,3),2),Raffled_depots(depot,3))
    case 4
        fprintf('#erratic( pose [%.3f %.3f 0 0] name "robot_0" color "blue") # node %d\n',node_list(Raffled_depots(depot,1),1),node_list(Raffled_depots(depot,1),2),Raffled_depots(depot,1))
        fprintf('#erratic( pose [%.3f %.3f 0 0] name "robot_1" color "red") # node %d\n',node_list(Raffled_depots(depot,2),1),node_list(Raffled_depots(depot,2),2),Raffled_depots(depot,2))
        fprintf('#erratic( pose [%.3f %.3f 0 0] name "robot_2" color "green") # node %d\n',node_list(Raffled_depots(depot,3),1),node_list(Raffled_depots(depot,3),2),Raffled_depots(depot,3))
        fprintf('#erratic( pose [%.3f %.3f 0 0] name "robot_3" color "yellow") # node %d\n',node_list(Raffled_depots(depot,4),1),node_list(Raffled_depots(depot,4),2),Raffled_depots(depot,4))
end
fprintf('\n')
end
fprintf('\n\n\n\n\n')


