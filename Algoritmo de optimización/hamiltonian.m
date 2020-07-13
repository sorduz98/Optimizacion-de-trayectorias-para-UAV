%{
Optimizacion de la ruta a seguir a partir de los costos 
minimos de cada tramo entre los waypoints.

Requiere:
    waypoints = Matriz que contiene los waypoints que se 
                encuentran en el mapa.
                La matriz debe tener la siguiente forma:
                waypoints = [x1 y1 z1;
                             x2 y2 z2;
                             ... ;
                             xn yn zn]
    costos =    Matriz que contiene los costos minimos entre
                cada tramo, y su direccion especifica. La 
                matriz debe tener la siguiente forma:
                costos = [w1 cost1 wx;
                          w2 cost2 wx;
                          ... ;
                          wn costn wn]

Retorna:
    OptTraj =   Matriz que comprende la trayecoria optima de la
                mision. Contiene todos los waypoints a seguir
                en la trayectoria optima.
                OptTraj = [x1 y1 z1;
                           x2 y2 z2;
                           ... ;
                           xn yn zn]
    cost_rute = Energia consumida en la ruta optima.
%}

function [OptTraj, cost_rute] = hamiltonian(waypoints, costos)
    sz=size(waypoints,1);
    OptTraj=ones(size(waypoints));
    x=ones(sz);
    y=ones(sz);
    w=zeros(sz);
    for i=1:sz
        x(i,:)=i;
        y(:,i)=i;
    end
    for j=1:size(costos,1)
       w(costos(j,1),costos(j,3)) = costos(j,2);
    end
    DG = sparse(x,y,w); %Creacion de la matriz de dispersion,
    %relaciones entre todos los nodos
    g=digraph(DG);      %Grafica direccional
    NW = size(x,2); %Numero total de Wayponits
    % Construccion de todos los caminos posibles que atraviesen todos los nodos

    paths = perms(2:NW); %Combinaciones posibles del camino
    paths = [ones(size(paths, 1), 1) paths];%Añade el nodo inicial y final

    dist = NaN(size(paths, 1), 1); %Declara como NaN al vector que representa la distancia por ruta

    for ii=1:size(paths, 1)
        path = paths(ii, :);
        edgeID = findedge(g, path(1:end-1), path(2:end)); %Encuentra los tramos del grafo segun la ruta
        if all(edgeID ~= 0)
            dist(ii) = sum(g.Edges.Weight(edgeID));%Guarda la distancia si la ruta es posible
        end
    end
    [~, id] = min(dist); %id es el # de la trayectoria con menor costo
    cost_rute = min(dist); %dist son las diferentes distancias calculadas
    OptRuta = paths(id, :) %Camino a recorrer
    for k=1:size(OptRuta, 2)
        W_actual=OptRuta(k);
        OptTraj(k,:)= waypoints(W_actual,:);
    end
    p = plot(g, 'EdgeLabel', g.Edges.Weight);
    highlight(p, paths(id, :), 'EdgeColor', 'k', 'LineStyle', '--', 'Nodecolor', 'r', 'LineWidth',2, 'Markersize',8);
end