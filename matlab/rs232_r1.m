clear all;
clc;
delete(instrfind);
close all;

BUFF_SIZE=300;

s=serial('COM3', 'BaudRate', 115200,'Terminator','LF','TimeOut',20);
fopen(s);

data=[];
t=[];

osc_fig=figure;
hold on;
grid on;
box on;
xlabel('t');
ylabel('data');

j=1;
try
    for i=1:1:10000
        data_header=fscanf(s,'%s');
        switch data_header
            case "OSC"
                osc_data(i,:) = fscanf(s,'%f,%f,%f,%f,%f')';
                if ( i>1 )
                    plot([i-1 i],[osc_data(i-1,1) osc_data(i,1)],'b.-');
                    plot([i-1 i],[osc_data(i-1,2) osc_data(i,2)],'r.-');
                    plot([i-1 i],[osc_data(i-1,3) osc_data(i,3)],'c.-');
                    plot([i-1 i],[osc_data(i-1,4) osc_data(i,4)],'g.-');
                    plot([i-1 i],[osc_data(i-1,5) osc_data(i,5)],'k.-');
                end      
                xlim([i-50 i]);
                drawnow;                
        
            case "DAT"                
                    data(j,:) = fscanf(s,'%f,%c,%c,%c,%c,%c,%c,%c,%c,%f')';
                    j=j+1;
                    if(mod(j,50)==0)
                        disp(['adquired ' num2str(j) ' samples']);
                    end                       
                    if (j>=BUFF_SIZE)
                        break;
                    end
                    
            otherwise
                sprintf('Unknown header-> %s', data_header)
        end
    end

    t=data(:,1)/1e3;%time sent in milliseconds and converted to seconds
    t(t==0)=NaN;
    circ_buffer1=data(:,2);
    circ_buffer2=data(:,3);
    circ_buffer3=data(:,4);
    circ_buffer4=data(:,5);
    circ_buffer5=data(:,6);
    circ_buffer6=data(:,7);
    circ_buffer7=data(:,8);
    circ_buffer8=data(:,9);
    debug_data1=data(:,10);
    
    fclose(s);
    
    [min_t,pos_min_t]=min(t);
    t=           circshift(t,           -pos_min_t+2);
    circ_buffer1=circshift(circ_buffer1,-pos_min_t+2);
    circ_buffer2=circshift(circ_buffer2,-pos_min_t+2);
    circ_buffer3=circshift(circ_buffer3,-pos_min_t+2);
    circ_buffer4=circshift(circ_buffer4,-pos_min_t+2);
    circ_buffer5=circshift(circ_buffer5,-pos_min_t+2);
    circ_buffer6=circshift(circ_buffer6,-pos_min_t+2);
    circ_buffer7=circshift(circ_buffer7,-pos_min_t+2);
    circ_buffer8=circshift(circ_buffer8,-pos_min_t+2);
    debug_data1 =circshift(debug_data1, -pos_min_t+2);
    
catch Me
    Me.identifier
    Me.message
    Me.stack
    close all;
    disp("Error. Closing serial port...");
    fclose(s);
    return;
end


% Ready	eReady
% Running	eRunning (the calling task is querying its own priority)
% Blocked	eBlocked
% Suspended	eSuspended
% Deleted	eDeleted (the tasks TCB is waiting to be cleaned up)
% /* Task states returned by eTaskGetState. */
% typedef enum
% {
%     eRunning = 0,   /* A task is querying the state of itself, so must be running. */
%     eReady,         /* The task being queried is in a read or pending ready list. */
%     eBlocked,       /* The task being queried is in the Blocked state. */
%     eSuspended,     /* The task being queried is in the Suspended state, or is in the Blocked state with an infinite time out. */
%     eDeleted,       /* The task being queried has been deleted, but its TCB has not yet been freed. */
%     eInvalid        /* Used as an 'invalid state' value. */
% } eTaskState;

eRunning=0;
eReady=1;
eBlocked=2;
eSuspended=3;
eDeleted=4;
eInvalid=5;

circ_buffer1=-circ_buffer1+2;
circ_buffer2=-circ_buffer2+2;
circ_buffer3=-circ_buffer3+2;
circ_buffer4=-circ_buffer4+2;
circ_buffer5=-circ_buffer5+2;
circ_buffer6=-circ_buffer6+2;
circ_buffer7=-circ_buffer7+2;
circ_buffer8=-circ_buffer8+2;

% close all;
figure;
hold on;
grid on;
box on;
stairs(t,circ_buffer1+0);
stairs(t,circ_buffer2+5);
stairs(t,circ_buffer3+10);
stairs(t,circ_buffer4+15);
stairs(t,circ_buffer5+20);
% stairs(t,circ_buffer6+25);
% stairs(t,circ_buffer7+30);
% stairs(t,circ_buffer8+35);

% set(gca,'XTick',[0:0.100:max(t)]);
set(gca,'YTick',[-1 0 1 2  4 5 6 7  9 10 11 12  14 15 16 17  19 20 21 22  24 25 26 27  29 30 31 32]);
set(gca,'YTickLabel',{'Suspended','Ready','Blocked','Running','Suspended','Ready','Blocked','Running','Suspended','Ready','Blocked','Running','Suspended','Ready','Blocked','Running','Suspended','Ready','Blocked','Running','Suspended','Ready','Blocked','Running','Suspended','Ready','Blocked','Running'});
set(gca,'XMinorTick','on');
% ylim([-1 19]);
% legend('\tau_{PID}','\tau_{IMU}','\tau_{ToF}','\tau_{UDP}','\tau_{Supervision}','\tau_{ENCODER A}','\tau_{ENCODER B}');

%plot arrival and deadlines

% T1=20;
% D1=15;
% c1=5;
% 
% T2=50;
% D2=45;
% c2=15;
% 
% T3=100;
% D3=90;
% c3=25;
% 
% T4=200;
% D4=150;
% c4=25;
% 
% 
% % U=c1/T1+c2/T2+c3/T3+c4/T4+c5/T5;
% % 
% a1_vector=0:T1/1000:max(t);
% D1_vector=a1_vector+D1/1000;
% plot([a1_vector; a1_vector], [0*5+0+zeros(1,length(a1_vector)); 0*5+3+zeros(1,length(a1_vector))],'Color',[1 .6 .6]);
% plot([a1_vector], [0*5+3+zeros(1,length(a1_vector))],'^','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);
% plot([D1_vector; D1_vector], [0*5+2.5+zeros(1,length(D1_vector)); 0*5+3+zeros(1,length(D1_vector))],'Color',[1 .6 .6]);
% plot([D1_vector], [0*5+2.5+zeros(1,length(D1_vector))],'v','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);
% 
% a2_vector=0:T2/1000:max(t);
% D2_vector=a2_vector+D2/1000;
% plot([a2_vector; a2_vector], [1*5+0+zeros(1,length(a2_vector)); 1*5+3+zeros(1,length(a2_vector))],'Color',[1 .6 .6]);
% plot([a2_vector], [1*5+3+zeros(1,length(a2_vector))],'^','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);
% plot([D2_vector; D2_vector], [1*5+2.5+zeros(1,length(D2_vector)); 1*5+3+zeros(1,length(D2_vector))],'Color',[1 .6 .6]);
% plot([D2_vector], [1*5+2.5+zeros(1,length(D2_vector))],'v','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);
% 
% a3_vector=0:T3/1000:max(t);
% D3_vector=a3_vector+D3/1000;
% plot([a3_vector; a3_vector], [2*5+0+zeros(1,length(a3_vector)); 2*5+3+zeros(1,length(a3_vector))],'Color',[1 .6 .6]);
% plot([a3_vector], [2*5+3+zeros(1,length(a3_vector))],'^','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);
% plot([D3_vector; D3_vector], [2*5+2.5+zeros(1,length(D3_vector)); 2*5+3+zeros(1,length(D3_vector))],'Color',[1 .6 .6]);
% plot([D3_vector], [2*5+2.5+zeros(1,length(D3_vector))],'v','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);
% 
% a4_vector=0:T4/1000:max(t);
% D4_vector=a4_vector+D4/1000;
% plot([a4_vector; a4_vector], [3*5+0+zeros(1,length(a4_vector)); 3*5+3+zeros(1,length(a4_vector))],'Color',[1 .6 .6]);
% plot([a4_vector], [3*5+3+zeros(1,length(a4_vector))],'^','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);
% plot([D4_vector; D4_vector], [3*5+2.5+zeros(1,length(D4_vector)); 3*5+3+zeros(1,length(D4_vector))],'Color',[1 .6 .6]);
% plot([D4_vector], [3*5+2.5+zeros(1,length(D4_vector))],'v','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);
% 
% % a5_vector=0:T5/1000:max(t);
% % D5_vector=a5_vector+D5/1000;
% % plot([a5_vector; a5_vector], [4*5+0+zeros(1,length(a5_vector)); 3*5+3+zeros(1,length(a5_vector))],'Color',[1 .6 .6]);
% % plot([a5_vector], [4*5+3+zeros(1,length(a5_vector))],'^','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);
% % plot([D5_vector; D5_vector], [4*5+2.5+zeros(1,length(D5_vector)); 3*5+3+zeros(1,length(D5_vector))],'Color',[1 .6 .6]);
% % plot([D5_vector], [4*5+2.5+zeros(1,length(D5_vector))],'v','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);


legend('\tau_{1}','\tau_{2}','\tau_{3}','\tau_{4}');
xlim([min(t) max(t)]);
% ylim([-1 23]);


% xzoom=0.1;
% steps=500
% for (i=0:max(t)/steps:max(t)-max(t)/steps-xzoom)
%     xlim([i i+xzoom])
%     pause(0.01);
% end

% figure;
% hold on;
% grid on;
% box on;
% plot(t,debug_data1);
% legend('data');
