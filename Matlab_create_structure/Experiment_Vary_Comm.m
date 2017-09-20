% %Depots = [41 69 6 16 50 25 8 44 34 55 51 74 43 24 57 13 18 17 31 71]
% 
% b_1 = [103.0 92.3 38.6 64.2 154.1 89.1 50.6 322.4 111.1 264.5];
% 
% b_2 = [203.4 205.5 338.1 315.8 302.6 24.3 276.6 126.2 86.5 246.8 282.0 152.2 210.3 16.8 317.1 81.9 218.1 189.1 132.7 233.7];
% meet_2 = [1 1 1 1 1 0 1 1 0 1 1 0 0 0 0 0 0 1 1 1];
% b_2 = b_2(1:10);
% 
% b_3 = [126.4 18.4 117.3 73.2 33.3 167.8 135.7 298.0 94.1 324.8];
% meet_3 = [3 0 3 0 0 3 2 6 1 5];
% 
% %R_4 = [100.6 126.7 39.4 68.4 107.6 184.7 32.8 142.8 66.9 142.7 142.5 127.1 159.8 17.9 103.5 73.1 67.7 52.1 102.8 29.9];
% b_4 = [123.5 122.6 38.6 81.6 143.0 24.2 115.2 225.0 116.2 70.7];
% meet_4 = [2 2 0 2 2 0 2 1 2 1];
% 
% 
% 
% 
% 
% 
% 
% % length(R_1)
% % length(R_2)
% % length(R_3)
% % length(R_4)
% 
% 
% figure(10)
% x = [b_1;b_2;b_3;b_4];
% boxplot(x','Labels',{[char(946) ' = 1.5'],[char(946) ' = 2.5'],[char(946) ' = 3.5'],[char(946) ' = 4.5']})
% 
% 
% title('Variation of object position','FontSize',15)
% % xlabel('Number of robots','FontSize',15)
% ylabel('Time (s)','FontSize',15)
% % grid on
% 
% 





%% Experiment varying start positions also



% In the experiment were considered 3 robots with the same starting
% position
M = [324.6	289.8	340.9	324.5	287.3	308.7;
390.7	345.5	437	307.3	257.3	281.9;
313.3	307.8	293.6	275.7	209.4	260.2;
318.2	293.2	247	256.3	326	243.8;
445.9	352.8	277.6	276.8	255.6	277;
451.2	300.7	224.2	270	265.8	277.9;
403.4	295.3	244.7	284.4	258.6	296.1;
378.2	245	250.8	256.3	286.5	276.7;
432.9	257.8	271.7	255.9	254.1	261.2;
411.7	321.7	288.8	320.4	309.9	265.5];



figure(10)
boxplot(M,'Labels',{[char(946) ' = 0 m'],[char(946) ' = 1.0 m'],[char(946) ' = 1.5 m'],[char(946) ' = 2.5 m'],[char(946) ' = 3.5 m'],[char(946) ' = 4.5 m']})
title(sprintf('Influency of communication range %c',char(946)),'FontSize',15)
% xlabel('Number of robots','FontSize',15)
ylabel('Time for complete coverage (s)','FontSize',15)
% grid on

M = M(:,[1,2,5])
figure(100)
boxplot(M,'Labels',{[char(946) ' = 0 m'],[char(946) ' = 1.0 m'],[char(946) ' = 3.5 m']})
title(sprintf('Influency of communication range %c',char(946)),'FontSize',15)
% xlabel('Number of robots','FontSize',15)
ylabel('Time for complete coverage (s)','FontSize',15)
% grid on


