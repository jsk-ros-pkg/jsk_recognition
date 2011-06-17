if( exist('rosoct_findpackage','file') )
    libsiftfast_path = rosoct_findpackage('libsiftfast');
else
    libsiftfast_path = fullfile(pwd, '..','3rdparty','libsiftfast');
end

if( ~isempty(libsiftfast_path) )
    addpath(fullfile(libsiftfast_path,'share','siftfast','octave'));

%     if( isempty(rvision_siftfast_added) )
%         if( isunix() )
%             separator = ':';
%         else
%             separator = ';';
%         end
%         newpath = [getenv('PATH') separator fullfile(libsiftfast_path,'lib')];
%         if( exist('setenv','builtin' ) )
%            setenv('PATH',newpath);
%         else
%            putenv('PATH',newpath);
%         end
%         rvision_siftfast_added = 1;
%     end
else
    disp('failed to find libsiftfast');
end
