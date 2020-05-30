% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
close all

imagepath = './train';
Samples = [];
for k=1:15
    % Load image
    I = imread(sprintf('%s/%03d.png',imagepath,k));
    
    % You may consider other color space than RGB
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
    
    HSV = rgb2hsv(I);
    H = HSV(:,:,1);
    S = HSV(:,:,2);
    V = HSV(:,:,3);
    
    % Collect samples 
    disp('');
    disp('INTRUCTION: Click along the boundary of the ball. Double-click when you get back to the initial point.')
    disp('INTRUCTION: You can maximize the window size of the figure for precise clicks.')
    figure(1), 
    mask = roipoly(I); 
    figure(2), imshow(mask); title('Mask');
    sample_ind = find(mask > 0);
    
    H = H(sample_ind);
    S = S(sample_ind);
    V = V(sample_ind);
    
    Samples = [Samples; [H S V]];
    
    disp('INTRUCTION: Press any key to continue. (Ctrl+c to exit)')
    pause
end

% visualize the sample distribution
figure, 
scatter(Samples(:,1),Samples(:,2),'.');
title('Pixel Hue and Saturation Distribubtion');
xlabel('Hue');
ylabel('Saturation');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%

n = size(Samples,1);
mu = 1/n * sum([Samples(:,1) Samples(:,2)]);
Sigma = zeros(2,2);
for i = 1:n
    s = [Samples(i,1) Samples(i,2)] - mu;
    Sigma = Sigma + s'*s;
end
Sigma = Sigma/n;
