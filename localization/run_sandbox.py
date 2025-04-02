from sandbox import isolate
@isolate

def sandbox_main():
    from localization.motion_model import main
    main()


if __name__ == "__main__":
    sandbox_main()

