function P = generate_Pcpp()
%Loads the files written by the ICR::ObjectLOader and ICR::OWS classes and creates
%a compatible struct P for the Matlab icr-computation. DEBUG_OWS and DEBUG_OBJECT_LOADER
%need to be defined in the /icrcpp/include/debug.h header file prior to the 
%compilation of the icrcpp library

load('normals.txt');
load('points.txt');
load('wrenches.txt');
neighbors=textread('neighbors.txt','%s','delimiter','\n');

N=size(points,1);
L=size(wrenches,1)/N;
for i=1:N
    P(i).e=0;
    nb=strread(neighbors{i});
    P(i).NbOfNeighbors=length(nb);
    P(i).nb_i=nb;
    P(i).p = points(i,:)'; 
    P(i).n = normals(i,:)'; 
    P(i).w = wrenches(L*(i-1)+1:i*L,:)';
    P(i).cf=wrenches(L*(i-1)+1:i*L,1:3)';
end
P(1).tri=[];
P(1).t_max=[];
