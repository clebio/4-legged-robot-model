import click

from src.simulation import imu, walking

@click.group()
def cli():
    pass

@cli.group()
def simulate():
    pass

@simulate.command()
def walk():
    walking.walk()

@simulate.command()
def imu():
    imu.stand()


@cli.group()
def run():
    pass

@run.command()
def angles():
    from src.real import angles

@run.command()
def zero():
    from src.real import zero_servos

@run.command()
def calibrate():
    from src.real import calibration

if __name__ == '__main__':
    cli()