% %
% Common Variables
% clear all; close all;
% warning('off');
% L = 450;
% W = 700;
% image = zeros(L,W);
% 
% dX = -40;
% dY = 7;
% 
% i = 3;
function [image] = Numbers(image,i,dX,dY,brightness)
    if i == 0
        %Number 0
        image(25+dY:50+dY,575+dX:578+dX) = brightness;
        image(25+dY:26+dY,560+dX:575+dX) = brightness;
        image(48+dY:50+dY,560+dX:578+dX) = brightness;
        image(25+dY:50+dY,560+dX:563+dX) = brightness;
    elseif i == 1
        %Number 1
        image(25+dY:50+dY,575+dX:578+dX) = brightness;
    elseif i == 2
        %Number 2
        image(25+dY:37+dY,575+dX:578+dX) = brightness;
        image(25+dY:26+dY,560+dX:575+dX) = brightness;
        image(48+dY:50+dY,560+dX:578+dX) = brightness;
        image(38+dY:39+dY,560+dX:578+dX) = brightness;
        image(38+dY:50+dY,560+dX:563+dX) = brightness;
    elseif i == 3
        %Number 3
        image(25+dY:50+dY,575+dX:578+dX) = brightness;
        image(25+dY:26+dY,560+dX:575+dX) = brightness;
        image(48+dY:50+dY,560+dX:578+dX) = brightness;
        image(38+dY:39+dY,560+dX:578+dX) = brightness;
    else
        %Number 4
        image(25+dY:50+dY,575+dX:578+dX) = brightness;
        image(38+dY:39+dY,560+dX:578+dX) = brightness;
        image(25+dY:38+dY,560+dX:562+dX) = brightness;
    end
    %H
    image(25+dY:50+dY,597+dX:600+dX) = brightness;
    image(25+dY:50+dY,611+dX:614+dX) = brightness;
    image(37+dY:38+dY,600+dX:611+dX) = brightness;

    %A
    image(25+dY:50+dY,624+dX:627+dX) = brightness;
    image(25+dY:50+dY,638+dX:641+dX) = brightness;
    image(25+dY:26+dY,627+dX:638+dX) = brightness;
    image(37+dY:38+dY,627+dX:638+dX) = brightness;

    %N
    image(25+dY:50+dY,651+dX:654+dX) = brightness;
    image(25+dY:50+dY,665+dX:668+dX) = brightness;

    image(25+dY,654+dX:655+dX) = brightness;
    image(26+dY,654+dX:655+dX) = brightness;
    image(27+dY,654+dX:655+dX) = brightness;
    image(28+dY,655+dX:656+dX) = brightness;
    image(29+dY,655+dX:656+dX) = brightness;
    image(30+dY,656+dX:657+dX) = brightness;
    image(31+dY,656+dX:657+dX) = brightness;
    image(32+dY,657+dX:658+dX) = brightness;
    image(33+dY,657+dX:658+dX) = brightness;
    image(34+dY,658+dX:659+dX) = brightness;
    image(35+dY,658+dX:659+dX) = brightness;
    image(36+dY,659+dX:660+dX) = brightness;
    image(37+dY,659+dX:660+dX) = brightness;
    image(38+dY,660+dX:661+dX) = brightness;
    image(39+dY,660+dX:661+dX) = brightness;
    image(40+dY,661+dX:662+dX) = brightness;
    image(41+dY,661+dX:662+dX) = brightness;
    image(42+dY,662+dX:663+dX) = brightness;
    image(43+dY,662+dX:663+dX) = brightness;
    image(44+dY,663+dX:664+dX) = brightness;
    image(45+dY,663+dX:664+dX) = brightness;
    image(46+dY,664+dX:665+dX) = brightness;
    image(47+dY,664+dX:665+dX) = brightness;
    image(48+dY,665+dX:666+dX) = brightness;
    image(49+dY,665+dX:666+dX) = brightness;
    image(50+dY,665+dX:666+dX) = brightness;

    %D
    image(25+dY:50+dY,678+dX:681+dX) = brightness;
    image(27+dY:48+dY,690+dX:693+dX) = brightness;
    image(25+dY:26+dY,681+dX:690+dX) = brightness;
    image(49+dY:50+dY,681+dX:690+dX) = brightness;

end

%imagesc(image,[0 255]);
%colorbar;