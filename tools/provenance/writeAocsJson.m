function writeAocsJson(file, value)
%WRITEAOCSJSON Write a human-readable report and fail explicitly on I/O errors.
fid = fopen(file, 'w');
if fid < 0
    error("AOCS:Tools:WriteFailed", "Cannot write %s", file);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', jsonencode(value, PrettyPrint=true));
end
