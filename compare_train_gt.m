i = 295; load('train_labels.mat');
I = imread(sprintf('data/train/pos/%s', frames{i}));
[M, N, ~] = size(I);

imshow([I I]);
rectangle('Position', lhd(i,:), 'EdgeColor', 'green', 'LineWidth', 2, 'Curvature', 0.1);
rectangle('Position', rhd(i,:), 'EdgeColor', 'magenta', 'LineWidth', 2, 'Curvature', 0.1);
rectangle('Position', lhp(i,:), 'EdgeColor', 'magenta', 'LineWidth', 2, 'Curvature', 0.1);
rectangle('Position', rhp(i,:), 'EdgeColor', 'cyan', 'LineWidth', 2, 'Curvature', 0.1);

i = 1; load('train_v1_labels.mat');
rectangle('Position', lhd(i,:) + [N 0 0 0], 'EdgeColor', 'red', 'LineWidth', 2, 'Curvature', 0.1);
rectangle('Position', rhd(i,:) + [N 0 0 0], 'EdgeColor', 'green', 'LineWidth', 2, 'Curvature', 0.1);
rectangle('Position', lhp(i,:) + [N 0 0 0], 'EdgeColor', 'magenta', 'LineWidth', 2, 'Curvature', 0.1);
rectangle('Position', rhp(i,:) + [N 0 0 0], 'EdgeColor', 'cyan', 'LineWidth', 2, 'Curvature', 0.1);

title('Detected hands (left) vs. ground truth (right)');
