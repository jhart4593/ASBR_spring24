addpath('./CommonFunctions/')
%% Parse PA-1 Files
PA1DataPath = './HW3-PA1/';

fileList = dir(PA1DataPath);

vecChars = 'a':'k';

for i = 1:size(vecChars,2)
    charHeader(1,i) = {vecChars(i)};
end

rowNames = {'auxilliary1';'calbody';'calreadings';'empivot';'optpivot';'output1';'estPostPosEM';'estPostPosOM'};

dataArray = cell(numel(rowNames)+1,numel(charHeader)+1);
dataArray(2:end,1) = rowNames;
dataArray(1,2:end) = charHeader;
for i = 1:size(fileList,1)
    fileName = split(fileList(i).name,'-')';
    
    if strcmp(fileName{1},'pa1')
        letterIndx = double(fileName{3})-double('a')+1;
        if strcmp(fileName{4},'auxilliary1.txt')
        elseif strcmp(fileName{4},'calbody.txt')
            % Parse CalBody File
            CalBodyData = importdata([PA1DataPath fileList(i).name]);
            CalHeader = split(CalBodyData.textdata,',');
            CalHeader = cellfun(@str2num,CalHeader,'UniformOutput',false);
            numOM_EMBase = CalHeader{1}; 
            numOM_CalObj = CalHeader{2}; 
            numEM_CalObj = CalHeader{3};
            
            di = CalBodyData.data(1:numOM_EMBase,:);
            ai = CalBodyData.data(numOM_EMBase+1:numOM_EMBase+numOM_CalObj,:);
            ci = CalBodyData.data(numOM_EMBase+numOM_CalObj+1:numOM_EMBase+numOM_CalObj+numEM_CalObj,:);
            dataArray(3,letterIndx+1) = {{di,ai,ci}};

        elseif strcmp(fileName{4},'calreadings.txt')
            % Parse Cal Readings File
            CalReadingsData = importdata([PA1DataPath fileList(i).name]);
            CalHeader = split(CalReadingsData.textdata,',');
            CalHeader = cellfun(@str2num,CalHeader,'UniformOutput',false);
            numOM_EMBase = CalHeader{1}; 
            numOM_CalObj = CalHeader{2}; 
            numEM_CalObj = CalHeader{3};
            numFrames = CalHeader{4}';
            
            EM_markerPos = cell(numFrames,3);
            for j = 1:numFrames
            
                di = CalReadingsData.data(1:numOM_EMBase,:);
                ai = CalReadingsData.data(numOM_EMBase+1:numOM_EMBase+numOM_CalObj,:);
                ci = CalReadingsData.data(numOM_EMBase+numOM_CalObj+1:numOM_EMBase+numOM_CalObj+numEM_CalObj,:);
                CalReadingsData.data(1:numOM_EMBase+numOM_CalObj+numEM_CalObj,:) = [];
            
                EM_markerPos(j,:) = {di, ai, ci};
            end
            dataArray(4,letterIndx+1) = {EM_markerPos};
        elseif strcmp(fileName{4},'empivot.txt')
            % Parse Empivot
            EmpivotData = importdata([PA1DataPath fileList(i).name]);
            EmpHeader = split(EmpivotData.textdata,',');
            EmpHeader = cellfun(@str2num,EmpHeader,'UniformOutput',false);
            numEM_probe = EmpHeader{1}; 
            numFrames = EmpHeader{2};
            
            GCoor = cell(numFrames,1);
            for j = 1:numFrames
                gi = EmpivotData.data(1:numEM_probe,:);
                EmpivotData.data(1:numEM_probe,:) = []; 
            
                GCoor(j,:) = {gi};
            end
            dataArray(5,letterIndx+1) = {GCoor};

        elseif strcmp(fileName{4},'optpivot.txt') 
            % Parse Optpivot
            OptpivotData = importdata([PA1DataPath fileList(i).name]);
            OptHeader = split(OptpivotData.textdata,',');
            OptHeader = cellfun(@str2num,OptHeader,'UniformOutput',false);
            numOM_EMBase = OptHeader{1}; 
            numOM_probe = OptHeader{2}; 
            numFrames = OptHeader{3};
            
            OM_markerPos = cell(numFrames,2);
            for j = 1:numFrames
            
                di = OptpivotData.data(1:numOM_EMBase,:);
                hi = OptpivotData.data(numOM_EMBase+1:numOM_EMBase+numOM_probe,:);
                OptpivotData.data(1:numOM_EMBase+numOM_probe,:) = []; 
            
                OM_markerPos(j,:) = {di,hi};
            end
            dataArray(6,letterIndx+1) = {OM_markerPos};
        elseif strcmp(fileName{4},'output1.txt')
            % Parse Output File 
            OutputData = importdata([PA1DataPath fileList(i).name]);
            OutHeader = split(OutputData.textdata,',');
            OutHeader = cellfun(@str2num,OutHeader,'UniformOutput',false);
            numEM_CalObj = OutHeader{1}; 
            numFrames = OutHeader{2}; 
            
            estPostPosEM = OutputData.data(1,:);
            estPostPosOM = OutputData.data(2,:);
            
            OutputData.data(1:2,:) = [];
            
            CCoor = cell(numFrames,1);
            for j = 1:numFrames
                ci = OutputData.data(1:numEM_CalObj,:); 
                OutputData.data(1:numEM_CalObj,:) = []; 
            
                CCoor(j,:) = {ci};
            end
            dataArray(7,letterIndx+1) = {CCoor};
            dataArray(8,letterIndx+1) = {estPostPosEM};
            dataArray(9,letterIndx+1) = {estPostPosOM};
        end
    end
end

%% Part 3: 3D point set to 3D point set registration (debug set)
clear
load('dataArray.mat')

debugsets = 'a':'g';
errorThresh = .1; 
maxIter = 100000; 

% Iterate over all debug datasets
for i = 1:length(debugsets)
    % Unwrap data
    set = debugsets(i); 
    colIndx = strcmp(dataArray(1,:),set);
    rowIndx = strcmp(dataArray(:,1),'calbody');
   
    di = dataArray{rowIndx,colIndx}{1};
    ai = dataArray{rowIndx,colIndx}{2};
    ci = dataArray{rowIndx,colIndx}{3}; 
    
    rowIndx = strcmp(dataArray(:,1),'calreadings');
    numFrames = size(dataArray{rowIndx,colIndx},1);
    % Iterate over each frame
    for j = 1:numFrames
        % Unwrap data
        Frame = j; 
        rowIndx = strcmp(dataArray(:,1),'calreadings');
        Di = dataArray{rowIndx,colIndx}{Frame,1};
        Ai = dataArray{rowIndx,colIndx}{Frame,2};

        rowIndx = strcmp(dataArray(:,1),'output1');
        CiOutput = dataArray{rowIndx,colIndx}{Frame};
        
        Fd = ICP(di,Di,@svdCorrespondence,errorThresh,maxIter); % perform ICP to find Fd
        Fa = ICP(ai,Ai,@svdCorrespondence,errorThresh,maxIter); % perform ICP to find Fa
        
        % Compute C from Fd and Fa
        CiCalc = zeros(size(ci,1),3); 
        for k = 1:size(ci,1)
            CiTemp = Tinv(Fd)*Fa*[ci(k,:)';1];
            CiCalc(k,:) = CiTemp(1:3)'; 
        end

        % Store data and compute error metrics
        FaStore(i,j) = {Fa};
        FdStore(i,j) = {Fd};
        ErrorMatMax(i,j) = max(max(abs(CiOutput - CiCalc)));
        ErrorMatMean(i,j) = mean(mean(abs(CiOutput - CiCalc)));
    end
    i
end

% Compute Fd mean and print
for i = 1:size(FdStore,1)
    FdSum = zeros(4,4);
    for j = 1:size(FdStore,2)
        FdSum = FdSum+FdStore{i,j};
    end
    FdMean = FdSum/size(FdStore,2);

    mat = FdMean; 
    fprintf(['Fd mean (across all frames) for data set ' num2str(i) ' :\n']);
    for ii = 1:size(mat, 1)
        for jj = 1:size(mat, 2)
            if mat(ii,jj) == 0
                fprintf('%.0f ', mat(ii, jj));
            else
                fprintf('%.4f ', mat(ii, jj));
            end
        end
        fprintf('\n');
    end
    fprintf('\n');
end


% Compute Fa mean and print
for i = 1:size(FaStore,1)
    FaSum = zeros(4,4);
    for j = 1:size(FaStore,2)
        FaSum = FaSum+FaStore{i,j};
    end
    FaMean = FaSum/size(FaStore,2);

    mat = FaMean; 
    fprintf(['Fa mean (across all frames) for data set ' num2str(i) ' :\n']);
    for ii = 1:size(mat, 1)
        for jj = 1:size(mat, 2)
            if mat(ii,jj) == 0
                fprintf('%.0f ', mat(ii, jj));
            else
                fprintf('%.4f ', mat(ii, jj));
            end
        end
        fprintf('\n');
    end
    fprintf('\n');
end

%% Part 3: 3D point set to 3D point set registration 
clear
load('dataArray.mat')

debugsets = 'h':'k';
errorThresh = .1; 
maxIter = 100000; 

% Iterate over all debug datasets
for i = 1:length(debugsets)
    % Unwrap data
    set = debugsets(i); 
    colIndx = strcmp(dataArray(1,:),set);
    rowIndx = strcmp(dataArray(:,1),'calbody');
   
    di = dataArray{rowIndx,colIndx}{1};
    ai = dataArray{rowIndx,colIndx}{2};
    ci = dataArray{rowIndx,colIndx}{3}; 
    
    rowIndx = strcmp(dataArray(:,1),'calreadings');
    numFrames = size(dataArray{rowIndx,colIndx},1);
    % Iterate over each frame
    for j = 1:numFrames
        % Unwrap data
        Frame = j; 
        rowIndx = strcmp(dataArray(:,1),'calreadings');
        Di = dataArray{rowIndx,colIndx}{Frame,1};
        Ai = dataArray{rowIndx,colIndx}{Frame,2};
        
        Fd = ICP(di,Di,@svdCorrespondence,errorThresh,maxIter); % perform ICP to find Fd
        Fa = ICP(ai,Ai,@svdCorrespondence,errorThresh,maxIter); % perform ICP to find Fa
        
        % Compute C from Fd and Fa
        CiCalc = zeros(size(ci,1),3); 
        for k = 1:size(ci,1)
            CiTemp = Tinv(Fd)*Fa*[ci(k,:)';1];
            CiCalc(k,:) = CiTemp(1:3)'; 
        end

        % Store data and compute error metrics
        FaStore(i,j) = {Fa};
        FdStore(i,j) = {Fd};
        CiCalcStore(i,j) = {CiCalc};
    end
    i
end


%% Part 4: EM Pivot (debug set)

clear
load('dataArray.mat')

debugsets = 'a':'g';
errorThresh = .1; 
maxIter = 100000; 

% Iterate over all debug datasets
for i = 1:length(debugsets)

    % Unwrap data
    set = debugsets(i); 
    colIndx = strcmp(dataArray(1,:),set);
    rowIndx = strcmp(dataArray(:,1),'empivot');
    numFrames = size(dataArray{rowIndx,colIndx},1);
    A = [];
    b = []; 

    % Iterate over all frames
    for j = 1:numFrames
        Frame = j;
        % Unwrap data
        Gi = dataArray{rowIndx,colIndx}{Frame,1};

        if j == 1 % Find mean
            GCent = mean(Gi);
            gi = Gi -GCent; 
        end
        
        Fg = ICP(gi,Gi,@quatCorrespondence,errorThresh,maxIter); % Preform ICP to find Fg
        A(end+1:end+3,:) = [Fg(1:3,1:3) -eye(3)];
        b(end+1:end+3,:) = -Fg(1:3,4);
    end
    
    % Least Squares Solution
    [U,S,V] = svd(A); % A = U*S*V'
    S = round(S,10);
    Srecp = S; 
    Srecp(S~=0) = 1./S(S~=0);
    
    x = V*Srecp'*U'*b;
%     isequal(round(V*Srecp'*U',9),round(pinv(A),9));

    % Store results and compute error metric 
    xStore(i,:) = x';

    rowIndx = strcmp(dataArray(:,1),'estPostPosEM');
    estPostPosEM = dataArray{rowIndx,colIndx};
    EMPostError(i,1) = max(abs(x(4:end)'-estPostPosEM));
    i
end
%% Part 4: EM Pivot 

clear
load('dataArray.mat')

debugsets = 'h':'k';
errorThresh = .1; 
maxIter = 100000; 

% Iterate over all debug datasets
for i = 1:length(debugsets)

    % Unwrap data
    set = debugsets(i); 
    colIndx = strcmp(dataArray(1,:),set);
    rowIndx = strcmp(dataArray(:,1),'empivot');
    numFrames = size(dataArray{rowIndx,colIndx},1);
    A = [];
    b = []; 

    % Iterate over all frames
    for j = 1:numFrames
        Frame = j;
        % Unwrap data
        Gi = dataArray{rowIndx,colIndx}{Frame,1};

        if j == 1 % Find mean
            GCent = mean(Gi);
            gi = Gi -GCent; 
        end
        
        Fg = ICP(gi,Gi,@quatCorrespondence,errorThresh,maxIter); % Preform ICP to find Fg
        A(end+1:end+3,:) = [Fg(1:3,1:3) -eye(3)];
        b(end+1:end+3,:) = -Fg(1:3,4);
    end
    
    % Least Squares Solution
    [U,S,V] = svd(A); % A = U*S*V'
    S = round(S,10);
    Srecp = S; 
    Srecp(S~=0) = 1./S(S~=0);
    
    x = V*Srecp'*U'*b;
%     isequal(round(V*Srecp'*U',9),round(pinv(A),9));

    % Store results
    xStore(i,:) = x';
    i
end

%% Part 5: Opt Pivot (debug set)
clear
load('dataArray.mat')

debugsets = 'a':'g';
errorThresh = .1; 
maxIter = 100000; 

% Iterate over all debug datasets
for i = 1:length(debugsets)

    % Unwrap data
    set = debugsets(i); 
    colIndx = strcmp(dataArray(1,:),set);

    rowIndx = strcmp(dataArray(:,1),'calbody');
    di = dataArray{rowIndx,colIndx}{1};
    
    rowIndx = strcmp(dataArray(:,1),'optpivot');
    numFrames = size(dataArray{rowIndx,colIndx},1);
    A = [];
    b = []; 

    % Iterate over all frames
    for j = 1:numFrames
        Frame = j;
        % Unwrap data
        Di = dataArray{rowIndx,colIndx}{Frame,1};
        Hi = dataArray{rowIndx,colIndx}{Frame,2};
        Fd = ICP(di,Di,@svdCorrespondence,errorThresh,maxIter); % Preform ICP to find Fd
        
        % Rotate Data into EM Frame
        for k = 1:size(Hi,1)
            HiRot = Tinv(Fd)*[Hi(k,:)';1];
            Hi(k,:) = HiRot(1:3)';
        end
        if j == 1 % Find mean
            HCent = mean(Hi);
            hi = Hi -HCent; 
        end
        
        Fh = ICP(hi,Hi,@quatCorrespondence,errorThresh,maxIter); % Preform ICP to find Fh
        A(end+1:end+3,:) = [Fh(1:3,1:3) -eye(3)];
        b(end+1:end+3,:) = -Fh(1:3,4);
    end

    % Least Squares Solution
    [U,S,V] = svd(A); % A = U*S*V'
    S = round(S,10);
    Srecp = S; 
    Srecp(S~=0) = 1./S(S~=0);
    
    x = V*Srecp'*U'*b;
%     isequal(round(V*Srecp'*U',9),round(pinv(A),9));

    % Store results and compute error metric 
    xStore(i,:) = x';

    rowIndx = strcmp(dataArray(:,1),'estPostPosOM');
    estPostPosOM = dataArray{rowIndx,colIndx};
    OMPostError(i,1) = max(abs(x(4:end)'-estPostPosOM));
    i
end

%% Part 5: Opt Pivot 
clear
load('dataArray.mat')

debugsets = 'h':'k';
errorThresh = .1; 
maxIter = 100000; 

% Iterate over all debug datasets
for i = 1:length(debugsets)

    % Unwrap data
    set = debugsets(i); 
    colIndx = strcmp(dataArray(1,:),set);

    rowIndx = strcmp(dataArray(:,1),'calbody');
    di = dataArray{rowIndx,colIndx}{1};
    
    rowIndx = strcmp(dataArray(:,1),'optpivot');
    numFrames = size(dataArray{rowIndx,colIndx},1);
    A = [];
    b = []; 

    % Iterate over all frames
    for j = 1:numFrames
        Frame = j;
        % Unwrap data
        Di = dataArray{rowIndx,colIndx}{Frame,1};
        Hi = dataArray{rowIndx,colIndx}{Frame,2};
        Fd = ICP(di,Di,@svdCorrespondence,errorThresh,maxIter); % Preform ICP to find Fd
        
        % Rotate Data into EM Frame
        for k = 1:size(Hi,1)
            HiRot = Tinv(Fd)*[Hi(k,:)';1];
            Hi(k,:) = HiRot(1:3)';
        end
        if j == 1 % Find mean
            HCent = mean(Hi);
            hi = Hi -HCent; 
        end
        
        Fh = ICP(hi,Hi,@quatCorrespondence,errorThresh,maxIter); % Preform ICP to find Fh
        A(end+1:end+3,:) = [Fh(1:3,1:3) -eye(3)];
        b(end+1:end+3,:) = -Fh(1:3,4);
    end

    % Least Squares Solution
    [U,S,V] = svd(A); % A = U*S*V'
    S = round(S,10);
    Srecp = S; 
    Srecp(S~=0) = 1./S(S~=0);
    
    x = V*Srecp'*U'*b;
%     isequal(round(V*Srecp'*U',9),round(pinv(A),9));

    % Store results
    xStore(i,:) = x';
    i
end