clc
clear all
close all

date = '20171025';
run = '003';
recordpath = ['/Users/benjamin/ros/data/udpRecord/' date '/' run '/'];

load([recordpath 'UDPdata_' run]);
try 
     rxdDatagrams = rxdPackets;
end


for i = 1:1
    disp('FrameNumber = udpData{i}(1:4)'); disp(udpData{i}(1))
    FrameNumber = char(udpData{i}(1));
    disp('ItemsInBlock = udpData{i}(5)'); disp(udpData{i}(5))
    ItemsInBlock = udpData{i}(5)
    disp('ItemID = udpData{i}(6)'); disp(udpData{i}(6))
    ItemID = udpData{i}(6)
    disp('ItemsDataSize = udpData{i}(7:8)'); disp(udpData{i}(7:8))
    ItemsDataSize = udpData{i}(7)
    disp('ObjectName = udpData{i}(9:32)'); disp(udpData{i}(9:32))
    ObjectName = char(udpData{i}(9:32)) 
    disp('TransX = udpData{i}(33:40)'); disp(udpData{i}(33:40))
    TransX = char(udpData{i}(33:40)) 
    disp('TransY = udpData{i}(41:48)'); disp(udpData{i}(41:48))
    TransY = char(udpData{i}(41:48)) 
    disp('TransZ = udpData{i}(49:56)'); disp(udpData{i}(49:56))
    TransZ = char(udpData{i}(49:56)) 
    disp('RotX = udpData{i}(57:64)'); disp(udpData{i}(57:64))
    RotX = char(udpData{i}(57:64)) 
    disp('RotY = udpData{i}(65:72)'); disp(udpData{i}(65:72))
    RotY = char(udpData{i}(65:72)) 
    disp('RotZ = udpData{i}(73:80)'); disp(udpData{i}(73:80))
    RotZ = char(udpData{i}(73:80)) 
end




%% now process how much data was in packets
viconData = [];
    for i = 1 : rxdPackets
        % load packet preamble
        Datagram = udpData{i};
        FrameNumber = Datagram(1);
        numObjectsInDatagram = Datagram(5);
        
        % using packet preamble, process into object data
        offset = 5;
        
        
        for i = 1 : numObjectsInDatagram
            itemID = Datagram(offset+1)+1;
            viconData{itemID}.itemID = itemID;
            viconData{itemID}.itemDataSize = Datagram(offset+2);
            viconData{i}.itemName = char(Datagram(offset+3+[1:24]));
%             TransX = Datagram(offset+[1:8]);offset = offset + 8;
%             packetObjects{i}.TransX = TransX;
%             packetObjects{i}.TransXrad = 10*...
%                 TransX(1)/10^1 + TransX(2)/10^2 + TransX(3)/10^3 + TransX(4)/10^4 + ...
%                 TransX(5)/10^5 + TransX(6)/10^6 + TransX(7)/10^7 + TransX(8)/10^8;
%             packetObjects{i}.TransY = Datagram(offset+[1:8]);offset = offset + 8;
%             packetObjects{i}.TransZ = Datagram(offset+[1:8]);offset = offset + 8;
%             packetObjects{i}.RotX = Datagram(offset+[1:8]);offset = offset + 8;
%             packetObjects{i}.RotY = Datagram(offset+[1:8]);offset = offset + 8;
%             packetObjects{i}.RotZ = Datagram(offset+[1:8]);offset = offset + 8;
%             clear TransX TransY TransZ
%             clear RotX RotY RotZ
           offset = offset + viconData{itemID}.itemDataSize;
        end


    end
    
%     disp('packetObjects{1}.TransXrad'); disp(packetObjects{1}.TransXrad)