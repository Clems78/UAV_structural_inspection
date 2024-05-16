close all;
clc;


gm = fegeometry("Cube.stl");
model = femodel;
model.Geometry = gm;    
model = generateMesh(model);
geometry = model.Geometry;

% figure
% pdegplot(model,"FaceAlpha",0.3, FaceLabels="on")
hold on
pdemesh(model,  ElementLabels="off");

% Find element associated with face 2
Ef2 = findElements(model, "region", Face =2);


% pdegplot(gm,CellLabels="on", FaceLabels="on", EdgeLabels="on", FaceAlpha=0.3);

% % figure
% % mesh = generateMesh(gm, Hmin=10)
% % pdemesh(mesh, ElementLabels="on");


% Limit the plot
% xlim([0,10]);
% ylim([0,10]);
% zlim([0,10]);
