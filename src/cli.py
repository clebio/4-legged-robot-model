import click


@click.group()
def cli():
    pass


@cli.group()
def simulate():
    from src.simulation import imu, walking


@cli.group()
def concurrent():
    pass


@cli.command()
def serve():
    from src.web import app

    app.main()


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


@run.command()
def servo_reset():
    from src.utils import reset_servos

    reset_servos()


@concurrent.command()
def simulate():
    from src.multi import simulation

    simulation.main()


@concurrent.command()
def calibrate():
    from src.multi import calibration

    calibration.main()


if __name__ == "__main__":
    cli()
