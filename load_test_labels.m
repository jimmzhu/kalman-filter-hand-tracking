%for i = 1:5
%  label_save_filename = sprintf('v%d_labels.mat', i);
%  label_dir = sprintf('v%d', i);
%  label_prefix = sprintf('%dL_', i);
%  [lhd, rhd, lhp, rhp] = format_data_labels(label_dir, label_prefix);
%
%end

if (false)
  label_save_filename = 'train_labels.mat';
  label = importdata('data/train/detections.txt', ' ');
  [N, ~] = size(label.data);

  % lhd rhd lhp rhp
  hands = -1*ones(N,4,4);
  frame_index = 0;
  frame_name = '';
  frame_names = cell(1, N);
  for i = 1:N
    if ~strcmp(frame_name, label.textdata{i})
      frame_name = label.textdata{i};
      hand_index = 1;
      frame_index = frame_index + 1;
      frame_names{frame_index} = frame_name;
    end
    hands(frame_index, :, hand_index) = label.data(i, 1:4);
    hand_index = hand_index + 1;
  end

  lhd = hands(1:frame_index,:,1); rhd = hands(1:frame_index,:,2);
  lhp = hands(1:frame_index,:,3); rhp = hands(1:frame_index,:,4);
  frames = frame_names(1:frame_index);
  save(label_save_filename, '-mat7-binary', 'lhd', 'rhd', 'lhp', 'rhp', 'frames');

  return;
end

for v = [1 4 5 6 7 8 10 14]
  label_save_filename = sprintf('test_v%d_labels.mat', v);
  label = importdata(sprintf('test_v%d_labels.txt', v), ' ');
  [N, ~] = size(label.data);

  % lhd rhd lhp rhp
  hands = -1*ones(N,4,4);
  frame_index = 0;
  frame_name = '';
  frame_names = cell(1, N);
  for i = 1:N
    if ~strcmp(frame_name, label.textdata{i})
      frame_name = label.textdata{i};
      hand_index = 1;
      frame_index = frame_index + 1;
      frame_names{frame_index} = frame_name;
    end
    hands(frame_index, :, hand_index) = label.data(i, 1:4);
    hand_index = hand_index + 1;
  end

  lhd = hands(1:frame_index,:,1); rhd = hands(1:frame_index,:,2);
  lhp = hands(1:frame_index,:,3); rhp = hands(1:frame_index,:,4);
  frames = frame_names(1:frame_index);
  save(label_save_filename, '-mat7-binary', 'lhd', 'rhd', 'lhp', 'rhp', 'frames');
end
