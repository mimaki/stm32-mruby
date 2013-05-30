MRuby::Toolchain.new(:stm32f4) do |conf|
  # C compiler settings
  [conf.cc, conf.cxx, conf.objc, conf.asm].each do |cc|
    cc.command = ENV['CC'] || 'arm-none-eabi-gcc'
    cc.flags = [ENV['CFLAGS'] || %w(
        -mcpu=cortex-m4
        -mthumb
        -mthumb-interwork
        -mlittle-endian
        -mfloat-abi=softfp
        -Os
        -Wl,--no-warn-mismatch
      )]
  end

  # Archiver settings
  conf.archiver do |archiver|
    archiver.command = ENV['AR'] || 'arm-none-eabi-ar'
  end
end
