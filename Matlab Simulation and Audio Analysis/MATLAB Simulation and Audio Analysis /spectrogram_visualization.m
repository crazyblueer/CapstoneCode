filenames = {'whatshesaid.m4a', 'we were away a year ago_lrr.wav'};
window_lengths = [25, 25]; 
fft_size = 1024; 
dyn_range = 80; % dB 
color_option = 'jet'; 
plot_spectrograms(filenames, window_lengths, fft_size, dyn_range, color_option);

function plot_spectrograms(filenames, window_lengths, fft_size, dyn_range, color_option)
    num_spectrograms = length(filenames);
    
    figure('Units', 'normalized', 'OuterPosition', [0 0 1 0.5]);
    
    for i = 1:num_spectrograms
        [x, Fs] = audioread(filenames{i});
        
        win_length = round(window_lengths(i) * 1e-3 * Fs);
        window = hamming(win_length);
        
        if size(x, 2) > 1
            x = x(:, 1); 
        end

        [S, F, T, P] = spectrogram(x, window, round(win_length * 0.5), fft_size, Fs, 'yaxis');
        
        P_dB = 10 * log10(P);
        
        P_dB = max(P_dB, max(P_dB(:)) - dyn_range);
        
        F_limit = F(F <= 4000);
        P_dB = P_dB(1:length(F_limit), :);
        
        subplot(1, num_spectrograms, i);
        imagesc(T, F_limit, P_dB);
        axis xy;
        xlabel('Time (s)');
        ylabel('Frequency (Hz)');
        title(['Spectrogram of ' filenames{i}]);
        colorbar;
        
        if strcmp(color_option, 'gray')
            colormap(gray);
        else
            colormap(jet);
        end
        caxis([-dyn_range 0]); 
        set(gca, 'FontSize', 12); 
        pbaspect([2 1 1]); 
    end
end
