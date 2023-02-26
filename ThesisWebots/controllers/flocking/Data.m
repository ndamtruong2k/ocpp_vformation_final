clc;
clear all;

fileID = fopen('robot_testEntro.txt','r');
fileID1 = fopen('robot1_testEntro.txt','r');
fileID2 = fopen('robot2_testEntro.txt','r');
fileID3 = fopen('robot3_testEntro.txt','r');
fileID4 = fopen('robot4_testEntro.txt','r');
fileID5 = fopen('robot5_testEntro.txt','r');
fileID6 = fopen('robot6_testEntro.txt','r');
fileID7 = fopen('robot7_testEntro.txt','r');
formatSpec = '%f %f %f %f %f';
sizeA = [5 Inf];
A = fscanf(fileID,formatSpec,sizeA);
A1 = fscanf(fileID1,formatSpec,sizeA);
A2 = fscanf(fileID2,formatSpec,sizeA);
A3 = fscanf(fileID3,formatSpec,sizeA);
A4 = fscanf(fileID4,formatSpec,sizeA);
A5 = fscanf(fileID5,formatSpec,sizeA);
A6 = fscanf(fileID6,formatSpec,sizeA);
A7 = fscanf(fileID7,formatSpec,sizeA);

A = A';
A1 = A1';
A2 = A2';
A3 = A3';
A4 = A4';
A5 = A5';
A6 = A6';
A7 = A7';

A = A(:,1:2);
A1 = A1(:,1:2);
A2 = A2(:,1:2);
A3 = A3(:,1:2);
A4 = A4(:,1:2);
A5 = A5(:,1:2);
A6 = A6(:,1:2);
A7 = A7(:,1:2);

A(:,3) = 1;
A1(:,3) = 1;
A2(:,3) = 1;
A3(:,3) = 1;
A4(:,3) = 1;
A5(:,3) = 1;
A6(:,3) = 1;
A7(:,3) = 1;

entro = [];
for i = 1:1:3677
    X = [A(i,:); A1(i,:);A2(i,:);A3(i,:);A4(i,:);A5(i,:);A6(i,:);A7(i,:)];
    entro = [entro ; entropytest(X);];
end
figure;
fx= figure
xA = 1:1:3677;
plot(xA,entro,'-ro');
xlim([0 3677]);
title('Entropy')
xlabel('Bước thời gian')
ylabel('Entropy')
figname=['Coverage Percentage'];
figname=fullfile(path,figname);
set(fx,'PaperPosition',[-0.2 0 6.5 4]);
set(fx,'PaperSize',[6.0 4]);
saveas(fx,'Entropy_Stage.pdf');