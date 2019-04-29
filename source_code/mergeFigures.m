function mergeFigures()
% merge saved figures into one subplot figure

disp('---------------------------------------------------------------------');
disp('note that all desired .fig files should contain only one figure inside it.');
disp('note that all desired .fig files should be defined in the 2D space.');
disp('Please press enter to continue');
pause
% Number of the run
N=0;
answer='y';
while strcmpi(answer,'y')
    N=N+1;
    % Get the path and filename of the desired fig file
    filename=0;
    run=0;
    while isequal(filename,0)
        if run==2
            error('please choose your fig file');
        end
        disp('---------------------------------------------------');
        disp('Select the desired fig file which you want to insert in the subplot: ');
        [filename,pathname]=uigetfile({'*.fig';'*.FIG'},'Select the .fig file you want to insert in the subplot');
        run=run+1;
    end
    clear FN
    FN=[pathname filename];
    % open figure
    h(N) = openfig(FN,'new');
    % get handle to axes of figure
    ax(N)=gca;
    answer=input('Do you have more .fig files to read? \n','s');
end
K=input('How many figures in a row do you want to have? \n');
if isempty(K)
    K=2;
end
figure;
for i=1:N
    % create and get handle to the subplot axes
    s(i) = subplot(ceil(N/K),K,i); 
    % get handle to all the children in the figure
    aux=get(ax(i),'children');
    for j=1:size(aux)
        fig(i) = aux(j);
        copyobj(fig(i),s(i)); 
        hold on
    end
    % copy children to new parent axes i.e. the subplot axes
    xlab=get(get(ax(i),'xlabel'),'string');
    ylab=get(get(ax(i),'ylabel'),'string');
    tit=get(get(ax(i),'title'),'string');
    leg=get(get(ax(i),'legend'));
    xlabel(xlab);ylabel(ylab);title(tit);
    box on; grid on;
end


% figure;
% for i = 1:length(varargin)
%     a(i) = hgload(varargin{i});
%     h(i) = subplot(length(varargin),1,i);
%     copyobj(allchild(get(a(i),'CurrentAxes')),h(i));
%     l(i) = legend(h(i),'LegendForFirstFigure');
% end