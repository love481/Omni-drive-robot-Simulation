function [connection] = simulation_setup()
    fileDir = fileparts(mfilename('fullpath')); % path to this m-file
    paths = fileDir; 
    % add the correct paths depending on the architecture
        libDir = [fileDir, '/../libs/matlab'];
        if(strcmp(computer,'GLNXA64'))
            paths = [paths, ':', fileDir, '/matlab'];  
            paths = [paths, ':', libDir, '/linuxLibrary64Bit'];
        elseif(strcmp(computer,'GLNX8632'))
            paths = [paths, ':', fileDir, '/matlab'];  
            paths = [paths, ':', libDir, '/linuxLibrary32Bit'];
        elseif(strcmp(computer,'PCWIN'))
            paths = [paths, ';', fileDir, '/matlab'];  
            paths = [paths, ';', libDir, '/windowsLibrary32Bit'];
        elseif(strcmp(computer,'PCWIN64'))
			paths = [paths, ';', fileDir, '/matlab'];  
            paths = [paths, ';', libDir, '/windowsLibrary64Bit'];
        elseif(strcmp(computer,'MACI64'))
			paths = [paths, ':', fileDir, '/matlab'];  
            paths = [paths, ':', libDir, '/macLibrary'];    
        else
            error('Not supported operation system detected');
        end

        addpath( paths );
        connection.vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)

end