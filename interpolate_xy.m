function [ inter_x, inter_y ] = interpolate_xy(x_coeff , y_coeff )
if(x_coeff(2)<x_coeff(1))
    inter_x = x_coeff(1);
    x_coeff(1) = x_coeff(2);
    x_coeff(2) = inter_x;
    inter_y = y_coeff(1);
    y_coeff(1) = y_coeff(2);
    y_coeff(2) = inter_y;   
end

slope = (y_coeff(1) - y_coeff(2)) / ( x_coeff(1) - x_coeff(2));
        

if (slope == Inf  || slope == -Inf)
    y_coeff = sort(y_coeff,'ascend');
    inter_y = y_coeff(1):y_coeff(2);
    for j=1:1:size(inter_y,2)
        inter_x(j) = x_coeff(1);
    end    
    
    
elseif(slope == 0)
    inter_x = x_coeff(1):x_coeff(2);
    for j=1:1:size(inter_x,2)
        inter_y(j) = y_coeff(1); 
    end
    

% elseif( abs(x_coeff(2)-x_coeff(1))<5)
%     y_coeff = sort(y_coeff,'ascend');
%     intercept = y_coeff(2) - slope*x_coeff(2);
%     inter_y = y_coeff(1):y_coeff(2);
%     inter_x = (inter_y(1,:)-intercept)/slope;
    
else
    intercept = y_coeff(2) - slope*x_coeff(2);
    inter_x = x_coeff(1):x_coeff(2);    
    inter_y = round(slope*inter_x(1,:) + intercept);    
    
end

% if(y_coeff(2)<y_coeff(1))
%     inter_y = fliplr(inter_y);
% end
end

