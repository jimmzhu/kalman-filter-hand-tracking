for v = 1:5
  mat_filename = sprintf('train_v%d_labels_gt.mat', v);
  load(mat_filename);

  textdir = sprintf('../hog2/v%d/', v);
  textFilenames = dir([textdir, '*.txt']);
  frames = cell(1, length(textFilenames));
  for i = 1:length(textFilenames)
    frames{i} = strrep(textFilenames(i).name, '.txt', '.png');
  end

  save(mat_filename, '-mat7-binary', 'lhd', 'lhp', 'rhd', 'rhp', 'frames');
end
