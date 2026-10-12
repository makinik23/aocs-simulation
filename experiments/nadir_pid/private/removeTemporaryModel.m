function removeTemporaryModel(model, folder)
	if bdIsLoaded(model)
		close_system(model,0);
	end

	if isfolder(folder)
		rmdir(folder,'s');
	end
end

