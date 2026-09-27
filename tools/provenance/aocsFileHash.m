function hash = aocsFileHash(file)
%AOCSFILEHASH SHA-256 of a file, independent of platform line endings on disk.
fid = fopen(file, 'rb');
if fid < 0
    error("AOCS:Tools:ReadFailed", "Cannot read %s", file);
end
cleanup = onCleanup(@() fclose(fid));
digest = java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid)
    digest.update(fread(fid, 1048576, '*uint8'));
end
bytes = typecast(digest.digest(), 'uint8');
hash = string(lower(reshape(dec2hex(bytes, 2).', 1, [])));
end
