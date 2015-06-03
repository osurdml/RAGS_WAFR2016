clear all ;
close all ;
clc ;

obstacles = dlmread('obstacles.txt',',') ;
membership = dlmread('membership.txt',',') ;

figure
imshow(obstacles);

figure
imshow(membership/11) ;