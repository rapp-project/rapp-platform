

int main ( int argc, char* argv[] )
{
    try
    {
        rapp::services::service_controller ctrl = rapp::services::service_controller( "alex", "qwepoi" );
        ctrl.Scheduler().run();
        
        
    }
    catch (std::exception& e)
    {
        std::cout << "Exception: " << e.what() << "\n";
    }

    return 0;
}