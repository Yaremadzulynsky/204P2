%----------------------------------------
% MTE 204 - Project 2
% Project 2: 2D Truss
% Question: 2
%----------------------------------------

%----------------------------------------
% Truss Displacement Calculation
%----------------------------------------
% Note: All units are in N and m for calculations; output is in cm
format long; % Set the display format to show more digits

%----------------------------------------
% Input Parameters
%----------------------------------------
E = 207000; % Young's Modulus in MPa
% Cross-sectional stiffness values for each element
% Dimensions of the bar (converted to consistent units in cm)
r1 = 1.5; % Radius of element 1 (cm)
L1 = 10;  % Length of element 1 (cm)

r2 = 1.0; % Radius of element 2 (cm)
L2 = 15;  % Length of element 2 (cm)

r3 = 0.5; % Radius of element 3 (cm)
L3 = 20;  % Length of element 3 (cm)

% Cross-sectional areas of each element
A1 = pi * r1^2; % Cross-sectional area of element 1 (cm^2)
A2 = pi * r2^2; % Cross-sectional area of element 2 (cm^2)
A3 = pi * r3^2; % Cross-sectional area of element 3 (cm^2)

% Stiffness values for each element
k1 = (A1 * E) / L1; % Stiffness of element 1 (N/cm)
k2 = (A2 * E) / L2; % Stiffness of element 2 (N/cm)
k3 = (A3 * E) / L3; % Stiffness of element 3 (N/cm)

%----------------------------------------
% Global Stiffness Matrix Assembly
%----------------------------------------
% K1 is the global stiffness matrix, accounting for the connectivity of
% all elements and their contributions to the nodes.
K1 = [
    k1,     -k1,      0,      0;
   -k1,  k1 + k2,  -k2,      0;
     0,    -k2,  k2 + k3,  -k3;
     0,      0,    -k3,    k3;
];

%----------------------------------------
% Force Vector
%----------------------------------------
% F1 represents the external forces applied to the nodes.
% Here, only the last node (Node 4) has a force of 1000 N applied.
F1 = [0; 0; 0; 1000];

%----------------------------------------
% Apply Boundary Conditions
%----------------------------------------
% The first node is fixed, meaning its displacement is 0. This requires
% removing the first row and column of the stiffness matrix (K1) and the
% first value of the force vector (F1).
K1_mod = K1(2:end, 2:end); % Reduced stiffness matrix
F1_mod = F1(2:end);        % Reduced force vector

%----------------------------------------
% Solve for Displacements
%----------------------------------------
% Solve the system of linear equations using mldivide: K1_mod * X1_mod = F1_mod
% X1_mod contains the displacements of the free nodes (Nodes 2, 3, and 4).
% The mldivide method automatically selects the appropriate solver based on the matrix properties
% and therefore this will find the most effective method of solving the equation.
X1_mod = K1_mod \ F1_mod;

% Reconstruct the full displacement vector, adding 0 for the fixed node
X1 = [0; X1_mod];  % Full displacement vector (Node 1 displacement = 0)

% Convert the displacements from mm to cm for clarity
X1_cm = X1 / 10; % Convert mm to cm

%----------------------------------------
% Display Node Displacements
%----------------------------------------
disp('Displacements of Nodes (cm):');
for i = 1:length(X1_cm)
    fprintf('Node %d: %.7f cm\n', i, X1_cm(i));
end

% Display the displacement of the last node explicitly
disp('Displacement of Node 4 (cm):');
disp(X1_cm(4));

%----------------------------------------
% Plot Displacement vs Position
%----------------------------------------
positions = 0:length(X1)-1; % Node positions (indices starting from 0)
figure;
plot(positions, X1, '-o', 'LineWidth', 2, 'MarkerSize', 8);
title('Displacement vs. Position of Nodes');
xticks(0:3);                % Specify integer tick positions for x-axis
xticklabels(string(0:3));   % Explicitly set integer labels
xlabel('Position (Nodes)');
ylabel('Displacement (mm)');
grid on;

%----------------------------------------
% Explanation of Results
%----------------------------------------
% The displacements represent the movement of each node due to the applied
% force at Node 4. Node 1 remains fixed, so its displacement is zero.
% Nodes 2, 3, and 4 displace according to the flexibility of the connected
% elements and their stiffness.
