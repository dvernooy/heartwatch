 function FIR_test(x,y);
close all;
pkg load signal;

b = [172 321 579 927 1360 1858 2390 2916 3391 3768 4012 4096];
b = [b b(11:-1:1)]';
b1 = b;
a = 1;
fs = 50; %20 ms/sample = 50 samples/sec
[h,w]=freqz(b, a, 500, fs);
h = h./32768;
20*log10(abs(h));
plot(w,20*log10(abs(h)));
hold on;

%b = 24000*fir1(23,0.02);
%b = 46000*remez(22,[0 0.0079 0.186 1], [1 1 0 0]); %nominal 1.5Hz
%b = 46000*remez(22,[0 0.011 0.24 1], [1 1 0 0]); %2Hz
%b = 46000*remez(22,[0 0.015 0.255 1], [1 1 0 0]); %2.5Hz
%b = 46000*remez(22,[0 0.18 0.27 1], [1 1 0 0]); %3Hz

%{
%b = 46000*remez(22,[0 x y 1], [1 1 0 0]);
b = round(b);
b1 = [b1 b]
[h,w]=freqz(b, a, 500, fs);
h = h./(2^15);
20*log10(abs(h));
 plot(w,20*log10(abs(h)));

%}
b = 46000*remez(22,[0 0.011 0.24 1], [1 1 0 0]); %2Hz
b = round(b);
b1 = [b1 b]
[h,w]=freqz(b, a, 500, fs);
h = h./(2^15);
20*log10(abs(h));
 plot(w,20*log10(abs(h)));

b = 46000*remez(22,[0 0.015 0.255 1], [1 1 0 0]); %2.5Hz
b = round(b);
b1 = [b1 b]
[h,w]=freqz(b, a, 500, fs);
h = h./(2^15);
20*log10(abs(h));
 plot(w,20*log10(abs(h)));

b = 46000*remez(22,[0 0.018 0.27 1], [1 1 0 0]); %3Hz
b = round(b);
b1 = [b1 b]
[h,w]=freqz(b, a, 500, fs);
h = h./(2^15);
20*log10(abs(h));
 plot(w,20*log10(abs(h)));

%}
h=get(gcf, "currentaxes");
set(h, "fontsize", 12);
h = xlabel ("Frequency [Hz]");
set (h, "fontsize", 14) 
h = ylabel ("Magnitude (dB)");
set (h, "fontsize", 14)       

  
