close all; clear all;

%Parametros

k2=2509.5;
Kp=18;
kc=5;
kcl=239;
kox2=0.0000189;
kx=2;
bmax=0.000162;
kox=0.0388;
ks=416.6;
taus=6;
k4=1512;
k1=1.286;
dPG=0.55;
kp=2.0833; 
pgc=12;
h0=0.2;
a7=1.515;
b7=0.813;
aI=10;
a1=666.7;
a2=1500;
d1=0.2;
s1=0.0126;
K1=0.1;
s9=0.298;
k9=0.91;
b1=27780;
b2=0.02;
d2=1.2;
f=1;
kI=200;
k6=763.9;
k7=1000;
s3=0.529;
k3=1.6;
c1=67.73;
c2=12;
d3=1;
s4=0.0008;
k8=0.16;
ker=0.75;
cmax=1;



e=[1,0.15,0.1,0.25,0.1,0.1,0.2,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.25,0.5,0.8,1];

ttotal=18;
deltat=1;
vectorsize=ttotal/deltat;
ttotalr=18;
deltar=0.001;
vectorsizer=ttotalr/deltar;

% inicialização de vectores
p= zeros(1,vectorsize);
ox=zeros(1,vectorsize);
t=zeros(1,vectorsize);
c=zeros(1,vectorsize);
s=zeros(1,vectorsize);
or=zeros(1,vectorsize);
or2=zeros(1,vectorsize);
a=zeros(1,vectorsize);
pr=zeros(1,vectorsize);
pg=zeros(1,vectorsize);
pr2=zeros(1,vectorsize);
er=zeros(1,vectorsize);
er2=zeros(1,vectorsize);


% condiçoes iniciais

p(1)=0.1;
ox(1)=0.000010;
s(1)=0;
pg(1)=0.001;
c(1)=0.01;
a(1)=0;
pr(1)=0.000857;
pr2(1)=1.714;
er(1)=0.9;
er2(1)=0;
or(1)=0.000008;
or2(1)=0.08;
t(1)=0;



%%
for i=2:vectorsize
   
    t(i)=t(i-1)+deltat;
end
for i=2:vectorsize
    
    dp=k2*(1-(((pg(i-1).^2)/((Kp)^2+pg(i-1).^2))*((t(i-1).^4)/((kc)^4+t(i-1).^4))))*c(i-1)-(kcl*p(i-1));
    p(i)=p(i-1)+dp*deltat;
    
    dox=(kox2*pg(i-1)*s(i-1)*c(i-1))-(kx*ox(i-1))+(bmax*1/2*(1+sin(72*2*pi*t(i-1))));
    ox(i)=ox(i-1)+dox*deltat;
    
    ds=((-1)*(kox)*pg(i-1)*s(i-1)*c(i-1))+(ks*exp(-t(i-1)/taus));
    s(i)=s(i-1)+ds*deltat;
    
    dpg=(k4*or(i-1))-(dPG*pg(i-1));
    pg(i)=pg(i-1)+dpg*deltat;
end

for i=2:vectorsize

    if pg(i-1)-pgc>=0
        dc=(k1*c(i-1)*(cmax-c(i-1)))-(kp*c(i-1))*((t(i-1).^4)/((kc)^4+t(i-1).^4));
    else
        dc=(k1*c(i-1)*(cmax-c(i-1)));
    end
        
    %falta calcular dc, duvidas em como se poe a função
    
    c(i)=c(i-1)+dc*deltat; 
    
end

for i=2:vectorsize
    
    dpr=a1*pr2(i-1)*p(i-1)-(a2*pr(i-1))-(d1*pr(i-1));
    pr(i)=pr(i-1)+dpr*deltat;
    
    
    
    l=@(D) a7+pr(i-1)*exp(b7*D);

    valorintegral=integral(l,0,(i-1));
    
   
    if t(i)-5>=0
        da=exp((-b7)*t(i-1))*(valorintegral+h0);
    else
        da=1;
    end
    a(i)=a(i-1)+da*deltat;
    if (a(i)>(0.999*aI) && a(i)<(1.001*aI))
        tI=t(i);
    else
        tI=0;
    end
    
    dpr2=(-a1)*pr2(i-1)*p(i-1)+a2*pr(i-1)-d1*pr2(i-1)+((s1)/(1+K1*pr(i-1).^2))+((s9*er(i-1).^4)/(k9^4+er(i-1).^4));
    pr2(i)=pr2(i-1)+dpr2*deltat;
    
    der=b1*er(i-1)*e(i-1)-(b2*er(i-1))-(d2*er(i-1));
    er(i)=er(i-1)+der*deltat;
    
    if t(i-1)-tI>=0
        der2=(-b1)*er2(i-1)*e(i-1)+(b2*er(i-1))-(d2*er2(i-1))-((kI*er(i-1))*(1-exp((-f)*(t(i-1)-tI))))+((k6*er(i-1))/(k7+er(i-1)))+((s3)/(1+k3*pr(i-1)));
        
    else
        der2=(-b1)*er2(i-1)*e(i-1)+(b2*er(i-1))-(d2*er2(i-1))-((k6*er(i-1))/(K7+er(i-1)))+((s3)/(1+k3*pr(i-1)));
        
    end
    
     er2(i)=er2(i-1)+der2*deltat;
    
end
    

  
 

for i=2:vectorsize
    dor=(c1*or2(i-1)*ox(i-1))-(c2*or(i-1))-(d3*or(i-1));
    or(i)=or(i-1)+dor*deltat;
    dor2=((-c1)*or2(i-1)*ox(i-1))+(c2*or(i-1))-(d3*or2(i-1))+(s4)+((k8*er(i-1).^4)/(ker^4+er(i-1).^4));
    or2(i)=or2(i-1)+dor2*deltat;
    
end



subplot(2,3,1)
plot(t,p,'r');
xlabel('time(t)');
ylabel('concentration');


subplot(2,3,2)
plot(t,ox,'b');
xlabel('time(t)');
ylabel('concentration');

subplot(2,3,3)
plot(t,pg,'y');
xlabel('time(t)');
ylabel('concentration');

subplot(2,3,4)
plot(t,c,'m');
xlabel('time(t)');
ylabel('volume');

subplot(2,3,5)
plot(t,pr,'g');
xlabel('time(t)');
ylabel('concentration');

subplot(2,3,6)
plot(t,er,'k');
xlabel('time(t)');
ylabel('concentration');






    
            
            
    

