if (false)
  label_save_filename = 'train_labels_gt.mat';
  label_dir = 'data/train/posGt';
  label_prefix = '';
  [lhd, rhd, lhp, rhp] = format_data_labels(label_dir, label_prefix);

  save(label_save_filename, '-mat7-binary', 'lhd', 'rhd', 'lhp', 'rhp');
  return;
end

for i = 1:5
  label_save_filename = sprintf('v%d_labels_gt.mat', i);
  label_dir = sprintf('v%d', i);
  label_prefix = sprintf('%dL_', i);
  [lhd, rhd, lhp, rhp] = format_data_labels(label_dir, label_prefix);

  save(label_save_filename, '-mat7-binary', 'lhd', 'rhd', 'lhp', 'rhp');
end
