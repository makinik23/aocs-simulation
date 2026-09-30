function quoted = aocsShellQuote(value)
%AOCSSHELLQUOTE Quote one path argument for the host command interpreter.
value = string(value);
if ispc
    if any(contains(value, ["""", "%", newline, char(13)]))
        error("AOCS:Tools:UnsupportedPath", "Unsupported command path: %s", value);
    end
    quoted = """" + value + """";
else
    quoted = "'" + replace(value, "'", "'" + """" + "'" + """" + "'") + "'";
end
end
