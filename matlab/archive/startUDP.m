%% Start the echo server and create a UDP object.

date = '20171025';
run = '003';
recordpath = ['/Users/benjamin/ros/data/udpRecord/' date '/' run '/'];
% mkdir(recordpath);
cd(recordpath)

simUDP = false;
timeout = 2; %seconds
% timeout = 20; %seconds
% bufferSize = 256;
% bufferSize = 512;
bufferSize = 1024;
% trackerIP = '192.168.64.105';
trackerIP = '192.168.10.1';
trackerPort = 51001;
% trackerPort = 24001;
if simUDP
    %%
    echoudp('off')
    echoudp('on',51001)
    u = udp('saturn.local',51001);
    u.InputBufferSize = bufferSize; clear bufferSize
    set(u, 'timeout', timeout); clear timeout

    % Connect the UDP object to the host.
    fopen(u)

    % fwrite(u,[1:4,'a', 0, 65:74])
    for i = 1:5
        umsg = [...
            1; ... frameid 1
            0; ... frameid 2
            1; ... frameid 3
            2; ... items in block
            1; ... itemID
            7;2; ... itemDataSize
            round(10*rand(24,1)); ... itemName
            round(10*rand(8,1)); ... TransX
            round(10*rand(8,1)); ... TransY
            round(10*rand(8,1)); ... TransZ
            round(10*rand(8,1)); ... RotX
            round(10*rand(8,1)); ... RotY
            round(10*rand(8,1)); ... RotZ
            2; ... itemID
            7;2; ... itemDataSize
            round(10*rand(24,1)); ... itemName
            round(10*rand(8,1))/10; ... TransX
            round(10*rand(8,1)); ... TransY
            round(10*rand(8,1)); ... TransZ
            round(10*rand(8,1)); ... RotX
            round(10*rand(8,1)); ... RotY
            round(10*rand(8,1)); ... RotZ
            ];
        fwrite(u,umsg)
    end; clear i umsg
else
    u = udp(trackerIP,51002,'LocalPort', trackerPort); clear trackerIP trackerPort
    u.InputBufferSize = bufferSize; clear bufferSize
    u.Timeout = timeout; clear timeout

    % Connect the UDP object to the host.
    fclose(u)
    fopen(u)    
end

rxdDatagrams = 0;

DlgH = figure;
H = uicontrol('Style', 'PushButton', 'String', 'Break', 'Callback', 'delete(gcbf)');
while (ishandle(H))
    [Datagram, count, msg] = fread(u);
%     if (length(msg)==0) % then successful reading of udp
    if (count~=0) % then successful reading of udp
        rxdDatagrams = rxdDatagrams+1;
        disp('rxdDatagrams'); disp(rxdDatagrams)
        udpData{rxdDatagrams,1} = Datagram';
    else
        disp(msg)
        break
    end
end

clear Datagram count msg
echoudp('off')
fclose(u)
clear u
try
    save([recordpath 'UDPdata_' run], 'udpData', 'rxdDatagrams');
catch
    disp('issue with saving udp data')
end
    

clear date path run rxdDatagrams simUDP udpData  


