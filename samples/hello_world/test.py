import swd
dev = swd.Swd(swd_frequency=4000000)
print(dev.get_version())
