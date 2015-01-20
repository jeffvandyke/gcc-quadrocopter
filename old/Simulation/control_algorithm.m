function [quad] = control_algorithm(quad)

%     if quad.z == 0.01 % on ground
%         quad.pwm1 = quad.pwm1+3;
%         quad.pwm2 = quad.pwm2+3;
%         quad.pwm3 = quad.pwm3+3;
%         quad.pwm4 = quad.pwm4+3;
%     elseif quad.zv > 0 && quad.zv < .5 && quad.z < .7
%         quad.pwm1 = quad.pwm1+1;
%         quad.pwm2 = quad.pwm2+1;
%         quad.pwm3 = quad.pwm3+1;
%         quad.pwm4 = quad.pwm4+1;
%     elseif quad.zv > .5 && quad.z < 0.7
%         quad.pwm1 = quad.pwm1-2;
%         quad.pwm2 = quad.pwm2-2;
%         quad.pwm3 = quad.pwm3-2;
%         quad.pwm4 = quad.pwm4-2;
%     elseif quad.z > 1  && quad.zv > 0
%         quad.pwm1 = quad.pwm1-1;
%         quad.pwm2 = quad.pwm2-1;
%         quad.pwm3 = quad.pwm3-1;
%         quad.pwm4 = quad.pwm4-1;


%% Begin lift-off
if quad.z == 0.01
    quad.pwm1 = quad.pwm1+1;
    quad.pwm2 = quad.pwm2+1;
    quad.pwm3 = quad.pwm3+1;
    quad.pwm4 = quad.pwm4+1;
    quad.hover = 0;
end

if quad.za > 0 && quad.hover == 0;
    quad.hover = quad.pwm1;
    quad.pwm1 = quad.pwm1 + 5;
    quad.pwm2 = quad.pwm2 + 5;
    quad.pwm3 = quad.pwm3 + 5;
    quad.pwm4 = quad.pwm4 + 5;
    
end




if quad.direction > 0
    [quad.pwm1,quad.pwm2,quad.pwm3,quad.pwm4] = go_up(quad);
else
    [quad.pwm1,quad.pwm2,quad.pwm3,quad.pwm4] = go_down(quad);
end

    
% %     if abs(quad.zv > .25)
% %         quad.pwm1 = quad.pwm1-5*quad.zv;
% %         quad.pwm2 = quad.pwm2-5*quad.zv;
% %         quad.pwm3 = quad.pwm3-5*quad.zv;
% %         quad.pwm4 = quad.pwm4-5*quad.zv;
% %     elseif abs(quad.zv < 0.25)
% %         quad.pwm1 = quad.pwm1+1+.5*(1-quad.z);
% %         quad.pwm2 = quad.pwm2+1+.5*(1-quad.z);
% %         quad.pwm3 = quad.pwm3+1+.5*(1-quad.z);
% %         quad.pwm4 = quad.pwm4+1+.5*(1-quad.z);
% %     end
   
    
    quad.f1 = pwm2force(quad.pwm1);
    quad.f2 = pwm2force(quad.pwm2);
    quad.f3 = pwm2force(quad.pwm3);
    quad.f4 = pwm2force(quad.pwm4);
end

function [pwm1, pwm2, pwm3, pwm4] = go_up(quad)
pwm1 = quad.pwm1;    
pwm2 = quad.pwm2;
pwm3 = quad.pwm3;
pwm4 = quad.pwm4;
%% Up Reach Altitude
    if quad.z > quad.alt && quad.zv > 0;
        pwm1 = 127+quad.left;
        pwm2 = 127;
        pwm3 = 127+quad.right;
        pwm4 = 127;
    end

    if quad.zv < 0
       pwm1 = quad.hover-1+quad.left; 
       pwm2 = quad.hover-1; 
       pwm3 = quad.hover-1+quad.right; 
       pwm4 = quad.hover-1; 
       
        if quad.z < quad.alt
            pwm1 = quad.hover+3+quad.left;
            pwm2 = quad.hover+3;
            pwm3 = quad.hover+3+quad.right;
            pwm4 = quad.hover+3;
        end
    end
end
function [pwm1, pwm2, pwm3, pwm4] = go_down(quad)
pwm1 = quad.pwm1;    
pwm2 = quad.pwm2;
pwm3 = quad.pwm3;
pwm4 = quad.pwm4;   
%% Down Reach Altitude
    if quad.z < quad.alt && quad.zv < 0;
        pwm1 = 255-quad.right;
        pwm2 = 255;
        pwm3 = 255-quad.left;
        pwm4 = 255;
    end

    if quad.zv > 0
       pwm1 = quad.hover+1+quad.left;
       pwm2 = quad.hover+1;
       pwm3 = quad.hover+1+quad.right;
       pwm4 = quad.hover+1;
       
       
        if quad.z > quad.alt
            pwm1 = quad.hover-10+quad.left;
            pwm2 = quad.hover-10;
            pwm3 = quad.hover-10+quad.right;
            pwm4 = quad.hover-10;
        end
    end
end
    