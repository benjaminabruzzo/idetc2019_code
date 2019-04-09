disp('batching clas.m')
set(0, 'DefaultFigureVisible', 'off');

meta.date = '20180905/'; % gazebo experiment data

jstr = {...
    '003';...% [1.5 0 1.33 0]
    '006';...% [1.5 0 1.33 0]
    '010';...% [2.5 0 1.33 0]
    '016';...% [1.0 0 0.88 0]
    '018';...% [1.0 0 0.88 0]
    '019';...% [1.0 0 0.88 0]
    '020';...% [1.4 0 1.00 0]
    '022';...% [1.4 0 1.00 0]
    '023';...% [1.4 0 1.00 0]
    '024';...% [1.4 0 1.00 0]
    '025';...% [2.0 0 1.20 0]
    '026';...% [2.0 0 1.20 0]
    '028';...% [2.0 0 1.20 0]
    '034';...% [2.5 0 1.20 0]
    '042';...% [2.5 0 1.50 0]
    '046';...% [2.5 0 1.50 0]
    '048';...% [2.5 0 1.50 0]
    '049';...% [3.0 0 1.70 0]
    '051';...% [3.0 0 1.70 0]
    '052';...% [3.0 0 1.70 0]
    '053';...% [3.5 0 1.70 0]
    '055';...% [3.5 0 1.70 0]
    };
meta.dataroot = '/Users/benjamin/ros/data/';
meta.saveplots = false;
meta.saveirosplots = false;

for run = 1:length(jstr)
    clc
    meta.run = jstr{run};
    disp(['meta.run : ' meta.run])
    try 
        data = loadData(meta); 
    catch
        disp('loadData(meta) failed')
    end
    try 
        data = gen_metrics(data);
    catch
        disp('gen_metrics(data) failed')
    end
    
    try 
        meta.savedata_icra2019_picket = '/Users/benjamin/hast/tex/icra/icra2019/data/gazebo_picket/';
        save( [meta.savedata_icra2019_picket 'data_' [meta.date(1:end-1) '_'  meta.run] '.mat'] , 'data'); 
    catch
        disp('save(data) failed')
    end

    clear data
end

