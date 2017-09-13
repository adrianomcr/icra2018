



figure(3)


E0 = dlmread('./../distributed/text/visited_0.txt');
E1 = dlmread('./../distributed/text/visited_1.txt');
E2 = dlmread('./../distributed/text/visited_2.txt');
E3 = dlmread('./../distributed/text/visited_3.txt');


% E = [E0;E1;E2];
E = [E0;E1;E2;E3];


hist(E,40)