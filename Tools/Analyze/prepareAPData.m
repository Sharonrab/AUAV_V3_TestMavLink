function prepareAPData (filename)

eval(['perl noNansTelem.pl <' filename '>temp.csv']);

data = csvread('temp.csv', 2, 0);

save(filename(1:end-4), 'data');

delete('temp.csv');
