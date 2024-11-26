%----------------------------------------
% MTE 204 - Project 2
% Project 2: 2D Truss
% Question: 1
%----------------------------------------
k1=0.25; % Stiffness of element 1 (N/m)
k2=0.5;  % Stiffness of element 2 (N/m)
k3=1.5;  % Stiffness of element 3 (N/m)
k4=0.75; % Stiffness of element 4 (N/m)
k5=1;    % Stiffness of element 5 (N/m)
format long; % Set the display format to show more digits for displacements

%----------------------------------------
% Global Stiffness Matrix Assembly
%----------------------------------------
% K1 is the global stiffness matrix for the system.
% Each element's stiffness contributes to the matrix as determined by
% the connectivity of the nodes and the stiffness values of the elements.
K1 = [
    k1, -k1,    0,    0,    0,    0;
   -k1, k1+k2, -k2,    0,    0,    0;
     0, -k2,  k2+k3, -k3,    0,    0;
     0,   0,   -k3, k3+k4, -k4,    0;
     0,   0,     0,  -k4, k4+k5, -k5;
     0,   0,     0,    0,  -k5,  k5;
];

%----------------------------------------
% Force Vector
%----------------------------------------
% F1 represents the external forces applied at each node.
% Here, a force of 2 N is applied at node 6, and no forces are applied to other nodes.
F1 = [0; 0; 0; 0; 0; 2];

%----------------------------------------
% Apply Boundary Conditions
%----------------------------------------
% The first node is fixed, meaning its displacement is 0. To account for
% this, we remove the first row and column from the global stiffness matrix (K1)
% and the first value from the force vector (F1).
K1_mod = K1(2:end, 2:end); % Reduced stiffness matrix
F1_mod = F1(2:end);        % Reduced force vector

%----------------------------------------
% Solve for Displacements
%----------------------------------------
% Solve the system of linear equations using the mldivide method: K1_mod * X1_mod = F1_mod
% X1_mod contains the displacements of the free nodes (nodes 2 through 6).
% The mldivide method automatically selects the appropriate solver based on the matrix properties
% and therefore this will find the most effective method of solving the equation.
X1_mod = K1_mod \ F1_mod;
% Reconstruct the full displacement vector by including 0 for the fixed node
X1 = [0; X1_mod]; % Full displacement vector, where X1(1) = 0 (fixed node)

%----------------------------------------
% Display Results
%----------------------------------------
disp('Displacements of Nodes (m):');
for i = 1:length(X1)
    fprintf('Node %d: %.12f m\n', i, X1(i));
end

% Display the displacement of the last node (node 6) explicitly
disp('Displacement of node 6 (m):');
fprintf('Node 6: %.12f m\n', X1(6));

%----------------------------------------
% Plot Displacement vs Position
%----------------------------------------
% The positions correspond to the nodes (0 to 5 for 6 nodes).
positions = 0:length(X1)-1;

% Plot the displacements at each node
figure;
plot(positions, X1, '-o', 'LineWidth', 2, 'MarkerSize', 8); % Line with markers for clarity
title('Displacement vs. Position of Nodes');                % Plot title
xlabel('Position (Nodes)');                                 % X-axis label
ylabel('Displacement (m)');                                 % Y-axis label
grid on;                                                    % Enable grid for better readability
