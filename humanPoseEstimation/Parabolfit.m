clear 
close all

%set up range of position data
x = -50:50; 

%Simulation data
mu=20; %ground truth location of peak
y = -0.3*(x+mu).^2 + 200*randn(1,length(x)); %underlying function

%polyfit approach
%[p, S] = polyfit(x,y,2); 
%yf = polyval(p,x); 

%fit method
fitType = fittype('a*x^2 + b*x + c'); % The equation for your fit goes here
p0=[0 0 0];  %initial guess
f = fit(x',y',fitType, 'StartPoint', p0);
%make fit function based on fit parameters
yf=f.a*x.^2 + f.b*x + f.c;

%compute confidence interval
uncertainty = confint(f,0.90);

%compute delta for visualization
delta_abc=uncertainty(1,:)-uncertainty(2,:);

%Analytic function of peak location based on derivative of second order
%function
ydf=2*f.a*x+f.b;
%analytic solution is x=-.5*b/a
peakx=-.5*f.b/f.a;

%Computate systematic uncertainty of peak location based on error propagation from above equation
%See wheeler and ganji ch 7.2
delta_x=(-.5/f.a)*delta_abc(2)+(.5*f.b/f.a^2)*delta_abc(1);

plot(x,y,'o',x,yf,'-') 
hold on
legend('data','linear fit') 

plot(x, yf)
plot(x, ydf)

%visualize confidence interval of peak
yylim=ylim;
plot(peakx*[1 1], [min(yylim) max(yylim)], 'g')
plot((peakx+delta_x)*[1 1], [min(yylim) max(yylim)],'k')
plot((peakx-delta_x)*[1 1], [min(yylim) max(yylim)],'k')

