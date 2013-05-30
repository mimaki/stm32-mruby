##
# File Test

# File class

assert('File', 'class') do
  File.class == Class
end

assert('File', 'superclass') do
  File.superclass == Object
end

# constants

assert('File', 'SEEK_SET') do
  File::SEEK_SET == 0
end

assert('File', 'SEEK_CUR') do
  File::SEEK_CUR == 1
end

assert('File', 'SEEK_END') do
  File::SEEK_END == 2
end

# File.new

assert('File.new', 'few args') do
  e = nil
  begin
    File.new
  rescue ArgumentError => e # OK
  end
  e
end

assert('File.new', '(path)') do
  f1 = File.new("filename.ext")
end

assert('File.new', '(path, mode)') do
  f1 = File.new("file1", "r")
  f2 = File.new("file2", "r+")
  f3 = File.new("file3", "w")
  f4 = File.new("file4", "w+")
  f5 = File.new("file5", "a")
  f6 = File.new("file6", "a+")
  f1 && f2 && f3 && f4 && f5 && f6
end

assert('File.new', '(x, x)') do
  e1, e2 = nil, nil
  begin
    File.new(1)
  rescue TypeError => e1
  end
  begin
    File.new("a", 1)
  rescue TypeError => e2
  end
  e1 && e2
end

# File.open

assert('File.open', 'few args') do
  e = nil
  begin
    File.open
  rescue ArgumentError => e # OK
  end
  e
end

assert('File.open', '(path)') do
  f1 = File.open("filename.ext")
end

assert('File.open', '(path, mode)') do
  f1 = File.open("file1", "r")
  f2 = File.open("file2", "r+")
  f3 = File.open("file3", "w")
  f4 = File.open("file4", "w+")
  f5 = File.open("file5", "a")
  f6 = File.open("file6", "a+")
  f1 && f2 && f3 && f4 && f5 && f6
end

assert('File.open', '(x, x)') do
  e1, e2 = nil, nil
  begin
    File.open(1)
  rescue TypeError => e1
  end
  begin
    File.open("a", 1)
  rescue TypeError => e2
  end
  e1 && e2
end

# File#close

assert('File.close', '') do
  File.new("a").close == nil
end

# File#flush

assert('File.flush', '') do
  f = File.new("a")
  f.flush == f
end

# File#getc

assert('File.getc', '') do
  f = File.new("a")
  f.getc == "A"
end

# File#gets

assert('File.gets', 'no args') do
  f = File.new("a")
  f.gets == "A" * 127
end

assert('File.gets', '(rs)') do
  f = File.new("a")
  f.gets("\r\n") == "A" * 127
end

assert('File.gets', '(rs, limit)') do
  f = File.new("a")
  f.gets("\r", 255) == "A" * 255
end

assert('File.gets', '(x, x)') do
  e1, e2 = nil, nil
  f = File.new("a")
  begin
    f.gets(1) == ""
  rescue TypeError => e1
  end
  begin
    f.gets("_", "A")
  rescue TypeError => e2
  end
  e1 && e2
end

# File#putc

assert('File.putc', 'no arg') do
  e = nil
  begin
    File.new("a", "r+").putc
  rescue ArgumentError => e
  end
  e
end

assert('File.putc', '(Integer)') do
  File.new("a", "r+").putc(0x30) == 0x30
end

assert('File.putc', '(String)') do
  File.new("a", "r+").putc("A") == 0x41
end

assert('File.putc', 'type error') do
  e = nil
  f = File.new("a", "r+")
  begin
    f.putc(1.1)
  rescue TypeError => e
  end
  e
end

# File#puts

assert('File.puts', 'no arg') do
  File.new("a", "r+").puts == nil
end

assert('File.puts', '(Integer)') do
  File.new("a", "r+").puts(123) == nil
end

assert('File.puts', '(String)') do
  File.new("a", "r+").puts("aaa") == nil
end

assert('File.puts', '(*obj)') do
  File.new("a", "r+").puts(1, "2", [3, 4]) == nil
end

# File#read

assert('File.read', 'no args') do
  e = nil
  begin
    File.new("a").read
  rescue ArgumentError => e
  end
  e
end

assert('File.read', '(len)') do
  File.new("a").read(1) == "A"
end

assert('File.read', '(x)') do
  e = nil
  begin
    File.new("a").read("123")
  rescue TypeError => e
  end
  e
end

# File#write

assert('File.write', 'no arg') do
  e = nil
  begin
    File.new("a").write
  rescue ArgumentError => e
  end
  e
end

assert('File.write', '(String)') do
  File.new("a").write("1234567890") == 10
end

assert('File.write', '(x)') do
  e = nil
  begin
    File.new("a").write(123)
  rescue TypeError => e
  end
  e
end

# File#seek

assert('File.seek', 'no args') do
  e = nil
  begin
    File.new("a").seek
  rescue ArgumentError => e
  end
  e
end

assert('File.seek', '(offset)') do
  File.new("a").seek(999) == 0
end

assert('File.seek', '(offset, SEEK_SET)') do
  File.new("a").seek(100, File::SEEK_SET) == 0
end

assert('File.seek', '(offset, SEEK_CUR)') do
  File.new("a").seek(999, File::SEEK_CUR) == 0
end

assert('File.seek', '(offset, SEEK_END)') do
  File.new("a").seek(-123, File::SEEK_END) == 0
end

assert('File.seek', '(x)') do
  e1, e2 = nil, nil
  begin
    File.new("a").seek("NG")
  rescue TypeError => e1
  end
  begin
    File.new("b").seek(10, "NG")
  rescue TypeError => e2
  end
  e1 && e2
end

# File#size

assert('File.size', '') do
  File.new("a").size == 0
end
