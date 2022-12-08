N = 8;

ranges = [1, 2, 3];
max_height = [10, 10, 10];
steepness = [0, 1, .1]; 
density = [0, .1, .35];

for i = 1:length(ranges)
    rng(ranges(i));
    terr = Terrain(N, max_height(i), steepness(i), density(i));
    subplot(1, 3, i)
    s = terr.plot();
    set(gca,'XTick',[], 'YTick', [])
    title(sprintf('Steepness: %.2f, Density: %.2f', steepness(i), density(i)))
end