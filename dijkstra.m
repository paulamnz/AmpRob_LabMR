function [coste, ruta]=dijkstra(grafo, inicio, destino)

%Costes: cada fila del mapa de costes será una fila compuesta por el coste
%de dicho nodo y de qué nodo viene dicho coste
nnodos = size(grafo,1); %número de nodos
nodos_a_revisar = Inf(nnodos, 2); %nodos sin revisar. Cada fila corresponde a un nodo, por orden, y las columnas son el coste acumulado y el nodo del que viene
nodos_revisados = Inf(nnodos, 3);
nodo_actual = inicio;
nodos_a_revisar(inicio,1) = 0;
nodos_a_revisar(inicio,2) = inicio;
k = 1;% k es el índice que añade un nodo a los nodos revisados
fin = 0; %se pondrá a 1 cuando se hayan recorrido todos los caminos

nodos_expandidos = 0;

while(fin == 0)
    % Paso 1: miramos a qué nodos accede nuestro nodo 
    array_adyac = [];%array que contiene los nodos adyacentes al nodo actual
    for l = 1:1:nnodos
        if(grafo(nodo_actual,l) > 0) %si para la fila del nodo actual, una columna no está a 0 (o -1, que sería que a ese nodo ya se ha llegado)
            array_adyac = [array_adyac;l]; %guardamos los nodos a los que puede acceder
        end
    end

    %Paso 2: actualizamos los costes de dichos nodos
    for m = 1:1:size(array_adyac)
        %por un lado tenemos el coste de llegar a ese nodo adyacente por
        %otro lado
        coste_acum_nodo_adyac = nodos_a_revisar(array_adyac(m),1); %valor acumulado de la tabla sin revisar que corresponde a la fila donde está/nodo adyacente en cuestión

        %por el otro tenemos el coste de llegar al nodo actual + el coste
        %de llegar desde ese nodo hasta el nodo adyacente
        coste_grafo_nodo_adyac = grafo(nodo_actual, array_adyac(m));
        nuevo_coste = nodos_a_revisar(nodo_actual,1)+coste_grafo_nodo_adyac;
        if ( coste_acum_nodo_adyac >  nuevo_coste)
            nodos_a_revisar(array_adyac(m),1) = nuevo_coste; %actualizamos la tabla
            nodos_a_revisar(array_adyac(m),2) = nodo_actual; %ponemos de donde sale el coste
        end
    end
    %paso 3: marcamos el nodo como revisado
    nodos_expandidos = nodos_expandidos + 1;
    nodos_revisados(k,:) = [nodo_actual,nodos_a_revisar(nodo_actual,1), nodos_a_revisar(nodo_actual,2)];%el nodo que era, su coste, y de qué nodo venía
    k = k+1;
    nodos_a_revisar(nodo_actual,1) = -1;% para que el paso 1 lo descarte
    
    %paso 4: empezamos a revisar los otros nodos
    validos = nodos_a_revisar(:,1) >= 0; % excluimos los ya visitados
    if ~any(validos)
        fin = 1; % si no hay más nodos válidos, terminamos
    else
        indices_originales = find(validos); %nos da el numero (el nombre) de los nodos q cumplen la condición (validos es un array de true false entonces da el indice de los true)
        tabla_temporal = nodos_a_revisar(validos, :); %se queda con las filas donde validos es true
        [~, posicion_min] = min(tabla_temporal(:,1)); %el primer valor q devuelve es el valor mas pequeño, el segundo la posic
        nodo_actual = indices_originales(posicion_min); %miramos el indice real, ya que posicion_min se refiere a los no descartados
    end
    %hacemos paso 1, 2, 3 y 4 otra vez
end

fila_destino = find(nodos_revisados(:,1) == destino);
coste = nodos_revisados(fila_destino, 2);
ruta = destino;
nodo = destino;
while nodo ~= inicio
    fila = find(nodos_revisados(:,1) == nodo);
    nodo = nodos_revisados(fila, 3); % nodo del que viene
    ruta = [nodo, ruta];
end
fprintf('Nodos expandidos: %d\n', nodos_expandidos);
end

