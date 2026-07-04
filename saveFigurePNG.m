function saveFigurePNG(fig,filePath)
%SAVEFIGUREPNG Save a figure at publication-quality resolution.
set(fig,'PaperPositionMode','auto');
print(fig,filePath,'-dpng','-r300');
end
