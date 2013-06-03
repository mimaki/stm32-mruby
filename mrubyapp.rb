print "Hello, " * 3 + "mruby!\n"
pwm = PWM.new(3)  # D3 pin をPWM出力として使用
v = 0
d = 1
while true
  pwm.write(v)    # vの値を出力
  v += d
  if v == 0
    d = 1
    LED.on        # LEDを点灯
  elsif v == 255
    d = -1
    LED.off       # LEDを消灯
  end
  delay(10)       # 10msウェイト
end
