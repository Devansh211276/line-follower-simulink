s = tf('s');

G = (7.86*s - 145.5) / (s^2 + 38.37*s + 8.538e-12);

bode(G) % magnitude and phase response
grid on

margin(G) % Gain Margin, Phase Margin, Gain crossover frequency, Phase crossover frequency
[Gm,Pm,Wcg,Wcp] = margin(G);

disp("Gain Margin = ");
disp(Gm);

disp("Phase Margin = ");
disp(Pm);

disp("Phase crossover frequency = ");
disp(Wcg);

disp("Gain crossover frequency = ");
disp(Wcp);