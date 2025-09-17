#include <iostream>
#include <functional>

void save_with_free_fun(const std::string &file_name)
{
    std::cout << "Free Function: " << file_name << std::endl;
};

class FileSave
{
private:
    /* data */

public:
    FileSave(/* args */) = default;
    ~FileSave() = default;

    void save_with_member_fun(const std::string &file_name)
    {
        std::cout << "Member Function: " << file_name << std::endl;
    };
};

int main()
{
    FileSave file_save;

    auto save_with_lambda_fun = [](const std::string &file_name) -> void
    {
        std::cout << "Lambda Function: " << file_name << std::endl;
    };

    save_with_free_fun("Free_File.txt");
    file_save.save_with_member_fun("Member_File.txt");
    save_with_lambda_fun("Lambda_File.txt");

    std::function<void(const std::string &)> save1 = save_with_free_fun;
    std::function<void(const std::string &)> save3 = save_with_lambda_fun;
    // Member Function Packaging
    std::function<void(const std::string &)> save2 = std::bind(&FileSave::save_with_member_fun, file_save, std::placeholders::_1);

    save1("free_file.txt");
    save2("member_file.txt");
    save3("lambda_file.txt");

    return 0;
}