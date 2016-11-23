%% GENERAL IMAGE READ AND SIZE FUNCTION
clc;
close all;
clear all;
im = imread('track2.png');
im = im2bw(im);
[m n]= size(im);

%% GENERATION OF SAMPLES AND STATE WITH INTIAL STATE SPACE AND GOAL STATE SPACE 
figure, imshow(im);
hold on;
[m n z] = size(im);
start_pos = [31 101];
stop_pos = [721 361];

plot(start_pos(1),start_pos(2),'r.','MarkerSize',20)
text(start_pos(1)+2,start_pos(2)+2,'\leftarrow start')

plot(stop_pos(1),stop_pos(2),'g.','MarkerSize',20)
text(stop_pos(1)+2,stop_pos(2)+2,'\leftarrow stop')

plot([start_pos(1) stop_pos(1)],[start_pos(2) stop_pos(2),],'--r');

%%1=x_co-ord; 2=y_co-ord; 3=distance to nearest node; 4=distance to goal;
%%5=node connected to; 6=weight at that node;

unexp_count=1;
node_div = 10;
for i=1:node_div:n
    for j=1:node_div:m            
%             plot(i,j,'x', 'MarkerSize', 10);
            nodes(unexp_count,1:2) = [i,j];
            if(im(j,i)==0)
                nodes(unexp_count,6)=10000;
            else
                nodes(unexp_count,6)=1;
            end
%             unexplored(unexp_count,1)=unexp_count;
            unexp_count=unexp_count+1;
    end
end        

final_node = find(nodes(:,1)==stop_pos(1) & nodes(:,2)==stop_pos(2));

a = knnsearch(nodes(:,1:2),start_pos);
points = [start_pos(1) start_pos(2); stop_pos(1) stop_pos(2)];
dist = pdist(points,'euclidean');
nodes(a,3) = -1;   %% distance to itself
nodes(a,4) = dist; %% distance to goal
nodes(a,5) = -1;   %% node connected to
% unexplored(a)=[];
exp_count=1;
explored(exp_count,1)  = a;
exp_count=exp_count+1;

goal_flag = 1;
frontier_astar = [];
figure,imshow(im)
hold on;
while(goal_flag)
    x_loc = nodes(explored(exp_count-1),1); y_loc = nodes(explored(exp_count-1),2);
    pos = find( (nodes(:,1)<= x_loc+node_div & nodes(:,1)>=x_loc-node_div) & ...
            (nodes(:,2)<=y_loc+node_div & nodes(:,2)>=y_loc-node_div) & ...
            ~(nodes(:,1)==x_loc  & nodes(:,2)==y_loc)  );       %%look for 8 neighbours
           
    
    explored_pos = find(ismember(pos,explored)==1);      %% Removes nodes already explored
    if(~isempty(explored_pos))
        for remove_pos=size(explored_pos,1):-1:1
            pos(explored_pos(remove_pos))=[];
        end
    end
        
    dupli_pos = find(ismember(pos,frontier_astar)==1);  %% removes duplicate nodes from frontier
    if(~isempty(dupli_pos))
        for rem_dup=size(dupli_pos,1):-1:1
            pos(dupli_pos(rem_dup))=[];
        end
    end
    
    if(~isempty(pos))
        nodes(pos(:,1),5)=explored(exp_count-1);
    end
    
    frontier_astar = [frontier_astar;  pos];
    
    if(~isempty(pos))
        for i=1:1:size(pos,1)
            if(nodes(pos(i),3)==0)       
                points = [nodes(pos(i),1) nodes(pos(i),2); x_loc y_loc];
                trav_dist = pdist(points,'euclidean');
                nodes(pos(i),3)=trav_dist + nodes(explored(exp_count-1),3);
        
                points = [nodes(pos(i),1) nodes(pos(i),2); stop_pos(1) stop_pos(2)];
                goal_dist = pdist(points,'euclidean');
                nodes(pos(i),4)=goal_dist*nodes(pos(i),6);
            end     
        end
    end
    if(~isempty(frontier_astar))
        for i=1:1:size(frontier_astar,1)
            dist = nodes(frontier_astar(i),3) + nodes(frontier_astar(i),4);
            if(i==1)    
                min_dist  = dist;
                min_pos=i; 
            elseif(dist<min_dist)
                min_dist = dist;
                min_pos = i;
            end
        end
    
        explored(exp_count,1)=frontier_astar(min_pos);     
        exp_count=exp_count+1;
        plot(nodes(frontier_astar(min_pos),1), nodes(frontier_astar(min_pos),2) ,'x', 'MarkerSize', 10);
        pause(0.01)

        frontier_astar(min_pos)=[];    
         if(ismember(final_node,explored))
                goal_flag=0;
         end     
    end
end
% 
j=1;
current_node = explored(size(explored,1));

figure(3),imshow(im)
hold on;
while(1)
    current_loc = [nodes(current_node,1) nodes(current_node,2)];
    connected_node = nodes(current_node,5);
    if(connected_node == -1)
        break;
    end
    
    connected_loc = [nodes(connected_node,1) nodes(connected_node,2)];
    
    plot([current_loc(1) connected_loc(1)],[current_loc(2) connected_loc(2)],'g');
%     pause(0.5);
    current_node = connected_node;
end

plot(start_pos(1),start_pos(2),'r.','MarkerSize',20)
text(start_pos(1)+2,start_pos(2)+2,'\leftarrow start')

plot(stop_pos(1),stop_pos(2),'g.','MarkerSize',20)
text(stop_pos(1)+2,stop_pos(2)+2,'\leftarrow stop')
