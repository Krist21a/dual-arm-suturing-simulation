%% Five continuous stitches, 3 mm spacing; open the continuous scene and stop it.
root=fileparts(mfilename('fullpath'));
python=getenv('SUTURE_PYTHON');
if isempty(python)
    try
        environment=pyenv;
        python=char(environment.Executable);
    catch
        python='';
    end
    if isempty(python),python='python';end
end
runner=fullfile(root,'work','continuous_suture','run_native.py');
[status,message]=system(sprintf('"%s" "%s"',python,runner),'-echo');
assert(status==0,'Continuous suture failed: %s',message);
